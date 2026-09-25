function r = blind_snr(noisy, enhanced, fs)
%BLIND_SNR  No-reference SNR estimate for real recordings.
%
%   r = blind_snr(noisy, enhanced, fs)      (both time-aligned, same fs)
%
%   True SNR / STOI / PESQ need the clean speech, which does not exist for a
%   real battlefield recording.  This function gives the usual practical
%   substitute: frames are classified as speech / non-speech with an energy
%   VAD run on the (much cleaner) enhanced signal, and for each signal
%       SNR_est = 10 log10( (P_speech_frames - P_noise_frames) / P_noise_frames )
%   r.snr_in, r.snr_out : estimated SNR before / after (dB)
%   r.noise_reduction   : attenuation of the background in speech pauses (dB)
%   r.speech_frames     : fraction of frames classified as speech
    x = noisy(:);
    y = enhanced(:);
    n = min(numel(x), numel(y));
    x = x(1:n); y = y(1:n);
    N = round(0.032 * fs);
    nf = floor(n / N);
    Ex = mean(reshape(x(1:nf*N), N, nf).^2, 1)';
    Ey = mean(reshape(y(1:nf*N), N, nf).^2, 1)';
    Eys = sort(Ey);
    floor_y = mean(Eys(1:max(1, round(0.1 * nf))));        % quietest 10 %
    sp = Ey > max(floor_y, eps) * 10^(15/10);                % 15 dB above floor
    sp = logical(conv(double(sp), ones(3, 1), 'same'));      % hangover
    if ~any(sp) || all(sp)
        r = struct('snr_in', NaN, 'snr_out', NaN, 'noise_reduction', NaN, ...
                   'speech_frames', mean(sp));
        return;
    end
    est = @(E) 10 * log10(max(mean(E(sp)) - mean(E(~sp)), 1e-3 * mean(E(~sp))) / mean(E(~sp)));
    r.snr_in = est(Ex);
    r.snr_out = est(Ey);
    r.noise_reduction = 10 * log10(mean(Ex(~sp)) / max(mean(Ey(~sp)), eps));
    r.speech_frames = mean(sp);
end
