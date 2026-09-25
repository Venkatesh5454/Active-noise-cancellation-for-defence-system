function [noisy, noise_scaled, g] = mix_at_snr(clean, noise, snr_db)
%MIX_AT_SNR  Add noise to clean speech at a prescribed global SNR (dB).
%
%   [noisy, noise_scaled, g] = mix_at_snr(clean, noise, snr_db)
%
%   The noise is looped/cropped to the length of the clean signal and scaled
%   by g so that 10*log10(sum(clean.^2) / sum(noise_scaled.^2)) = snr_db.

    clean = clean(:);
    noise = noise(:);
    N = numel(clean);
    if numel(noise) < N
        noise = repmat(noise, ceil(N / numel(noise)), 1);
    end
    noise = noise(1:N);
    ps = sum(clean.^2);
    pn = sum(noise.^2);
    g = sqrt(ps / (pn * 10^(snr_db / 10)));
    noise_scaled = g * noise;
    noisy = clean + noise_scaled;
end
