function [e, w] = nlms_anc(d, xref, L, mu, vad_gate, fs)
%NLMS_ANC  Two-microphone adaptive noise cancellation (Widrow ANC, NLMS).
%
%   e = nlms_anc(d, xref, L, mu, vad_gate, fs)
%
%   d        : primary microphone   (speech + noise)
%   xref     : reference microphone (noise, picked up near the source)
%   L, mu    : adaptive FIR length and normalised step size (0 < mu < 2)
%   vad_gate : true -> freeze adaptation while speech dominates the primary
%              (prevents the filter from cancelling speech that leaks into
%              the reference microphone)
%
%   e : error signal = primary minus the adaptive estimate of the noise
%       path, i.e. the speech estimate passed to the spectral stage.

    d = d(:);
    xref = xref(:);
    n = numel(d);
    w = zeros(L, 1);
    xb = zeros(L, 1);
    e = zeros(n, 1);
    delta = 1e-6 * L * max(mean(xref.^2), eps);

    % speech-activity gate from the primary / reference power ratio
    adapt = true(n, 1);
    if vad_gate
        B = round(0.016 * fs);
        nb = floor(n / B);
        Ed = sum(reshape(d(1:nb*B).^2, B, nb), 1);
        Ex = sum(reshape(xref(1:nb*B).^2, B, nb), 1) + eps;
        r = 10 * log10(Ed ./ Ex + eps);
        base = movmin(r, round(1.0 * fs / B));         % noise-only ratio
        speech = r > base + 6;
        speech = movmax(speech, 3);                    % hangover
        adapt(1:nb*B) = ~reshape(repmat(speech, B, 1), [], 1);
    end

    for k = 1:n
        xb = [xref(k); xb(1:L-1)];
        e(k) = d(k) - w' * xb;
        if adapt(k)
            w = w + (mu * e(k) / (xb' * xb + delta)) * xb;
        end
    end
end
