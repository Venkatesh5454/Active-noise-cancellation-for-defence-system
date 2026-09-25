function m = snr_metrics(clean, est, fs)
%SNR_METRICS  Waveform-level quality measures against a clean reference.
%
%   m = snr_metrics(clean, est, fs)
%     m.snr    : global SNR (dB)          10log10(|s|^2 / |s - est|^2)
%     m.segsnr : segmental SNR (dB), 32 ms frames, each clamped to [-10, 35]
%     m.sisdr  : scale-invariant SDR (dB)
    s = clean(:);
    y = est(:);
    n = min(numel(s), numel(y));
    s = s(1:n);
    y = y(1:n);
    m.snr = 10 * log10(sum(s.^2) / max(sum((s - y).^2), eps));

    N = round(0.032 * fs); H = round(N / 2);
    starts = 1:H:(n - N + 1);
    v = zeros(numel(starts), 1);
    for i = 1:numel(starts)
        a = s(starts(i):starts(i)+N-1);
        e = a - y(starts(i):starts(i)+N-1);
        v(i) = min(max(10 * log10(sum(a.^2) / (sum(e.^2) + eps) + eps), -10), 35);
    end
    m.segsnr = mean(v);

    a = (y' * s) / max(s' * s, eps);
    t = a * s;
    m.sisdr = 10 * log10(sum(t.^2) / max(sum((y - t).^2), eps));
end
