function [y, fs_out, info] = defence_anc(x, fs, p)
%DEFENCE_ANC  Hybrid adaptive noise canceller for defence communications.
%
%   [y, fs_out, info] = defence_anc(x, fs)
%   [y, fs_out, info] = defence_anc(x, fs, anc_params('name', value, ...))
%
%   x  : noisy input.  One column = single microphone.  Two columns =
%        [primary, reference] microphones (the reference picks up mostly
%        noise); an NLMS adaptive canceller then runs before the
%        single-channel stage.  Two identical columns (dual-mono file) are
%        treated as one microphone.
%   fs : input sampling rate (any; processing runs at 16 kHz)
%
%   y      : enhanced speech at fs_out = 16 kHz, time-aligned with x
%   info   : processing statistics (latency, real-time factor, ...)
%
%   Processing chain
%     (1) resample to 16 kHz, DC blocker
%     (2) optional reference-microphone NLMS adaptive noise cancellation
%     (3) frame-by-frame spectral stage (anc_process_frame):
%         SPP-MMSE stationary-noise tracker, then either
%         - AI/ML stage (default): causal GRU network that estimates the
%           speech mask from the noisy spectrum + tracked noise PSD, trained
%           on speech mixed with gunshot / machine-gun / artillery /
%           helicopter / missile / drone / siren / vehicle noise, or
%         - statistical stage (no model file): impulsive and tonal noise
%           estimators + OM-LSA optimal gain
%         and a zero-phase spectral low-cut for rumble / wind.
%   The spectral stage is causal and processes 8 ms blocks with 32 ms
%   algorithmic latency, i.e. it can run block-synchronously on a DSP /
%   embedded board (see anc_realtime_demo.m).

    if nargin < 3 || isempty(p), p = anc_params(); end
    t_start = tic;

    x = double(x);
    if isvector(x), x = x(:); end
    mode = 'single-mic';
    if size(x, 2) >= 2
        a = x(:, 1); b = x(:, 2);
        c = sum(a .* b) / sqrt(sum(a.^2) * sum(b.^2) + eps);
        if c > 0.98 && sum((a - b).^2) < 1e-3 * sum(a.^2)
            x = mean(x(:, 1:2), 2);                % dual-mono file
        else
            mode = 'dual-mic';
        end
    end

    % ---- (1) resample + high-pass ----------------------------------------
    fs_out = p.fs;
    if strcmp(mode, 'single-mic')
        xr = resample_k(x(:, 1), p.fs, fs);
    else
        xr = [resample_k(x(:, 1), p.fs, fs), resample_k(x(:, 2), p.fs, fs)];
    end
    if p.dc_block
        % y[n] = x[n] - x[n-1] + a*y[n-1]   (cut-off ~10 Hz)
        a = exp(-2 * pi * 10 / p.fs);
        xr = filter([1 -1], [1 -a], xr);
    end

    % ---- (2) reference microphone NLMS ANC ------------------------------
    if strcmp(mode, 'dual-mic')
        d = nlms_anc(xr(:, 1), xr(:, 2), p.nlms_taps, p.nlms_mu, p.nlms_vad_gate, p.fs);
    else
        d = xr(:, 1);
    end

    % ---- (3) streaming spectral stage ------------------------------------
    if p.use_dnn && ~isfield(p, 'model')
        p.model = dnn_load(p.model_file);
    end
    st = anc_init(p);
    L = numel(d);
    delay = st.N - st.R;
    nhop = ceil((L + delay) / st.R);
    din = [d; zeros(nhop * st.R - L, 1)];
    yout = zeros(nhop * st.R, 1);
    for h = 1:nhop
        idx = (h-1) * st.R + (1:st.R);
        [yout(idx), st] = anc_process_frame(din(idx), st);
    end
    y = yout(delay + 1 : delay + L);

    pk = max(abs(y));
    if pk > p.out_peak
        y = y * (p.out_peak / pk);
    end

    info.mode            = mode;
    info.fs              = p.fs;
    info.latency_ms      = st.N / p.fs * 1000;
    info.hop_ms          = st.R / p.fs * 1000;
    info.frames          = nhop;
    info.impulsive_pct   = 100 * st.stat_tr_frames / nhop;
    info.tonal_pct       = 100 * st.stat_ton_frames / nhop;
    info.proc_time_s     = toc(t_start);
    info.rtf             = info.proc_time_s / (L / p.fs);
end

