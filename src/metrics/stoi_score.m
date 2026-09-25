function d = stoi_score(clean, processed, fs, extended)
%STOI_SCORE  Short-Time Objective Intelligibility (STOI) and extended STOI.
%
%   d = stoi_score(clean, processed, fs)          classic STOI  [1]
%   d = stoi_score(clean, processed, fs, true)    extended STOI [2]
%
%   clean     : clean reference speech (vector)
%   processed : noisy or enhanced speech, time-aligned with clean
%   fs        : sampling rate in Hz (signals are resampled to 10 kHz)
%
%   Returns a value in ~[0, 1]; higher = more intelligible.  Toolbox-free
%   re-implementation of the algorithm in [1]/[2]; it reproduces the
%   reference implementation (pystoi / Taal's MATLAB code) to within
%   numerical precision.
%
%   [1] C.H. Taal, R.C. Hendriks, R. Heusdens, J. Jensen, "An algorithm for
%       intelligibility prediction of time-frequency weighted noisy speech",
%       IEEE Trans. ASLP, 19(7), 2011.
%   [2] J. Jensen, C.H. Taal, "An algorithm for predicting the
%       intelligibility of speech masked by modulated noise maskers",
%       IEEE/ACM Trans. ASLP, 24(11), 2016.

    if nargin < 4, extended = false; end

    x = double(clean(:));
    y = double(processed(:));
    n = min(numel(x), numel(y));
    x = x(1:n);
    y = y(1:n);

    FS      = 10000;   % internal sampling rate
    NFRAME  = 256;     % window length (25.6 ms)
    NFFT    = 512;
    NUMBAND = 15;      % one-third octave bands
    MINFREQ = 150;     % centre of first band (Hz)
    N       = 30;      % frames per intermediate segment (384 ms)
    BETA    = -15;     % lower SDR bound (dB)
    DYN     = 40;      % speech dynamic range (dB)

    if fs ~= FS
        x = resample_k(x, FS, fs);
        y = resample_k(y, FS, fs);
    end

    [x, y] = remove_silent_frames(x, y, DYN, NFRAME, NFRAME/2);

    X = stdft(x, NFRAME, NFRAME/2, NFFT);
    Y = stdft(y, NFRAME, NFRAME/2, NFFT);
    if size(X, 2) < N
        warning('stoi_score:short', ...
            'Not enough speech frames for STOI (need >= %d). Returning NaN.', N);
        d = NaN;
        return;
    end

    OBM   = thirdoct(FS, NFFT, NUMBAND, MINFREQ);
    X_tob = sqrt(OBM * abs(X).^2);          % bands x frames
    Y_tob = sqrt(OBM * abs(Y).^2);

    nseg = size(X_tob, 2) - N + 1;
    J    = size(X_tob, 1);
    c    = 10^(-BETA / 20);
    dint = zeros(J, nseg);
    ext  = zeros(1, nseg);

    for m = 1:nseg
        Xs = X_tob(:, m:m+N-1);
        Ys = Y_tob(:, m:m+N-1);
        if extended
            xn = row_col_normalize(Xs);
            yn = row_col_normalize(Ys);
            ext(m) = sum(sum(xn .* yn)) / N;
        else
            alpha = sqrt(sum(Xs.^2, 2)) ./ (sqrt(sum(Ys.^2, 2)) + eps);
            Yp = min(Ys .* repmat(alpha, 1, N), Xs * (1 + c));
            Yp = Yp - repmat(mean(Yp, 2), 1, N);
            Xc = Xs - repmat(mean(Xs, 2), 1, N);
            Yp = Yp ./ repmat(sqrt(sum(Yp.^2, 2)) + eps, 1, N);
            Xc = Xc ./ repmat(sqrt(sum(Xc.^2, 2)) + eps, 1, N);
            dint(:, m) = sum(Xc .* Yp, 2);
        end
    end

    if extended
        d = mean(ext);
    else
        d = mean(dint(:));
    end
end

% -------------------------------------------------------------------------
function xn = row_col_normalize(x)
    xn = x - repmat(mean(x, 2), 1, size(x, 2));
    xn = xn ./ repmat(sqrt(sum(xn.^2, 2)) + eps, 1, size(x, 2));
    xn = xn - repmat(mean(xn, 1), size(x, 1), 1);
    xn = xn ./ repmat(sqrt(sum(xn.^2, 1)) + eps, size(x, 1), 1);
end

function w = hanning_sym(N)
    w = 0.5 * (1 - cos(2 * pi * (1:N)' / (N + 1)));   % = hanning(N)
end

function S = stdft(x, N, K, NFFT)
% Short-time DFT, frames start at 1:K:(length(x)-N), bins 0..NFFT/2.
    starts = 1:K:(numel(x) - N);
    if isempty(starts)
        S = zeros(NFFT/2 + 1, 0);
        return;
    end
    idx = repmat((0:N-1)', 1, numel(starts)) + repmat(starts, N, 1);
    fr  = x(idx) .* repmat(hanning_sym(N), 1, numel(starts));
    S   = fft(fr, NFFT, 1);
    S   = S(1:NFFT/2 + 1, :);
end

function [xs, ys] = remove_silent_frames(x, y, dyn, N, K)
% Drop frames whose clean-speech energy is > dyn dB below the loudest frame.
    starts = 1:K:(numel(x) - N);
    w   = hanning_sym(N);
    idx = repmat((0:N-1)', 1, numel(starts)) + repmat(starts, N, 1);
    xf  = x(idx) .* repmat(w, 1, numel(starts));
    yf  = y(idx) .* repmat(w, 1, numel(starts));
    e   = 20 * log10(sqrt(sum(xf.^2, 1)) + eps);
    keep = (e - max(e) + dyn) > 0;
    xf = xf(:, keep);
    yf = yf(:, keep);
    nk = size(xf, 2);
    if nk == 0
        xs = zeros(0, 1); ys = zeros(0, 1);
        return;
    end
    pos = repmat((1:N)', 1, nk) + repmat((0:nk-1) * K, N, 1);
    len = (nk - 1) * K + N;
    xs  = accumarray(pos(:), xf(:), [len 1]);
    ys  = accumarray(pos(:), yf(:), [len 1]);
end

function A = thirdoct(fs, nfft, num_bands, min_freq)
% One-third octave band matrix (bands x (nfft/2+1)).
    f  = linspace(0, fs, nfft + 1);
    f  = f(1:nfft/2 + 1);
    k  = 0:num_bands-1;
    fl = min_freq * 2.^((2*k - 1) / 6);
    fh = min_freq * 2.^((2*k + 1) / 6);
    A  = zeros(num_bands, numel(f));
    for i = 1:num_bands
        [~, lo] = min((f - fl(i)).^2);
        [~, hi] = min((f - fh(i)).^2);
        A(i, lo:hi-1) = 1;
    end
end
