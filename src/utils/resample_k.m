function y = resample_k(x, fs_out, fs_in)
%RESAMPLE_K  Toolbox-free polyphase resampler (Kaiser-windowed sinc, 60 dB).
%
%   y = resample_k(x, fs_out, fs_in) converts the column/row vector x from
%   sampling rate fs_in to fs_out (both integers).  The anti-aliasing
%   filter is the same Kaiser-windowed sinc design used by GNU Octave's
%   resample() (and by pystoi), so STOI/PESQ values computed with this
%   project match the reference Python implementations.  No Signal
%   Processing Toolbox is required.
%
%   Output length = ceil(length(x) * fs_out / fs_in), zero-phase aligned.

    if fs_out == fs_in
        y = x;
        return;
    end
    isrow_in = isrow(x);
    x = double(x(:));

    g = gcd(round(fs_out), round(fs_in));
    p = round(fs_out) / g;
    q = round(fs_in) / g;

    % ---- anti-aliasing filter (Kaiser-windowed ideal low-pass) ----------
    rejection_dB      = 60;
    stopband_cutoff_f = 1 / (2 * max(p, q));
    roll_off_width    = stopband_cutoff_f / 10;
    L    = ceil((rejection_dB - 8) / (28.714 * roll_off_width));
    t    = (-L:L)';
    arg  = 2 * stopband_cutoff_f * t;
    sincv = ones(size(arg));
    nz   = arg ~= 0;
    sincv(nz) = sin(pi * arg(nz)) ./ (pi * arg(nz));
    beta = 0.1102 * (rejection_dB - 8.7);
    M    = 2 * L + 1;
    n    = (0:M-1)';
    kais = besseli(0, beta * sqrt(1 - (2 * n / (M - 1) - 1).^2)) / besseli(0, beta);
    h    = kais .* (2 * p * stopband_cutoff_f * sincv);
    h    = p * h / sum(h);                  % exact unity DC gain after upsampling

    % ---- polyphase filtering: y[m] = sum_n x[n] h[m*q - n*p + L] ---------
    Lx = numel(x);
    Lh = numel(h);
    Ly = ceil(Lx * p / q);
    m    = (0:Ly-1)';
    tt   = m * q + L;
    nmax = floor(tt / p);
    r    = tt - nmax * p;                   % polyphase branch (0..p-1)
    J    = ceil(Lh / p);
    hp   = zeros(p * J, 1);
    hp(1:Lh) = h;
    H    = reshape(hp, p, J);               % H(r+1, j+1) = h(r + j*p + 1)

    y = zeros(Ly, 1);
    for j = 0:J-1
        nidx  = nmax - j;
        valid = (nidx >= 0) & (nidx <= Lx - 1);
        if ~any(valid), continue; end
        coef  = H(r(valid) + 1, j + 1);
        y(valid) = y(valid) + coef .* x(nidx(valid) + 1);
    end

    if isrow_in
        y = y.';
    end
end
