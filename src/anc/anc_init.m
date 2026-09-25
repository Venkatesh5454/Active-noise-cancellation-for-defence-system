function st = anc_init(p)
%ANC_INIT  Create the state of the streaming (real-time) noise canceller.
%
%   st = anc_init(anc_params())
%
%   The state holds the STFT buffers and all adaptive quantities.  Feed it
%   with anc_process_frame() one hop (8 ms = 128 samples @ 16 kHz) at a time.

    st.p   = p;
    st.N   = round(p.frame_ms * p.fs / 1000);          % 512
    st.R   = round(p.hop_ms * p.fs / 1000);            % 128
    st.K   = st.N / 2 + 1;                             % 257 bins
    n      = (0:st.N-1)';
    st.win = sqrt(0.5 - 0.5 * cos(2 * pi * n / st.N)); % sqrt-Hann (periodic)
    st.ola_norm = st.R / sum(st.win.^2);               % = 0.5 for 75 % overlap
    st.inbuf  = zeros(st.N, 1);
    st.outbuf = zeros(st.N, 1);
    st.frame  = 0;
    f = (0:st.K-1)' * p.fs / st.N;
    st.freq = f;

    % per-frame smoothing constants (parameters are given per 16 ms / 8 ms)
    hop_ratio = p.hop_ms / 16;
    st.a_psd  = p.spp_alpha_psd ^ hop_ratio;
    st.a_p    = p.spp_alpha_p ^ hop_ratio;
    st.n_init = max(1, round(p.init_ms / p.hop_ms));

    % SPP-MMSE noise tracker (Gerkmann & Hendriks, IEEE TASLP 2012)
    xi_h1 = 10^(p.spp_xi_h1_db / 10);
    st.prior_fact   = 1;                       % P(H0)/P(H1) = 1
    st.log_glr_fact = log(1 / (1 + xi_h1));
    st.glr_exp      = xi_h1 / (1 + xi_h1);
    st.lambda_s     = zeros(st.K, 1);
    st.PH1mean      = 0.5 * ones(st.K, 1);

    % transient (impulsive) noise stage
    st.tr_bins  = find(f >= p.tr_band_hz(1) & f <= p.tr_band_hz(2));
    st.a_ref    = exp(-p.hop_ms / p.tr_ref_ms);
    st.hold_dec = 10^(-p.tr_hold_db_s * p.hop_ms / 1000 / 10);
    st.tr_max   = round(p.tr_max_ms / p.hop_ms);
    st.Rref     = zeros(st.K, 1);
    st.Rhold    = zeros(st.K, 1);
    st.tr_count = 0;
    st.pt       = 0;

    % tonal stage
    % a continuously present peak reaches ton_thresh after ton_persist_ms
    st.a_ton    = (1 - p.ton_thresh) ^ (p.hop_ms / p.ton_persist_ms);
    st.T        = zeros(st.K, 1);
    st.ton_ok   = (f >= 150 & f <= 7800);
    st.med_idx  = min(max(repmat((1:st.K)', 1, 17) + repmat(-8:8, st.K, 1), 1), st.K);

    % zero-phase spectral low-cut (rumble, wind, helicopter rotor)
    st.lowcut = ones(st.K, 1);
    st.lowcut(f < p.lowcut_hz) = 10^(p.lowcut_db / 20);

    % gain stage
    st.a_dd   = p.dd_alpha;
    st.xi_min = 10^(p.xi_min_db / 10);
    st.qfac   = p.q_absence / (1 - p.q_absence);
    st.gmin   = 10^(p.gmin_db / 20);
    st.A2prev = zeros(st.K, 1);

    % AI/ML stage (GRU mask estimator), if a model is supplied
    st.dnn = [];
    if isfield(p, 'model') && ~isempty(p.model)
        st.dnn = p.model;
        st.h = zeros(st.dnn.H, 1);
        st.dnn_floor = 10^(p.dnn_floor_db / 20);
    end

    % statistics for reporting
    st.stat_tr_frames = 0;
    st.stat_ton_frames = 0;
end
