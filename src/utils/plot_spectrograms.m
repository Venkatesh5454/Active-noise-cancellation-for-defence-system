function plot_spectrograms(x, y, fs, ttl)
%PLOT_SPECTROGRAMS  Noisy vs. enhanced spectrogram (same colour scale).
    if nargin < 4, ttl = ''; end
    [Sx, t, f] = spec_db(x, fs);
    Sy = spec_db(y, fs);
    top = max(Sx(:));
    figure('Name', ['ANC: ' ttl], 'Color', 'w');
    subplot(2, 1, 1);
    imagesc(t, f, Sx); axis xy; caxis([top - 80, top]); colormap(jet); colorbar;
    title(['Noisy input: ' ttl], 'Interpreter', 'none'); ylabel('Frequency (kHz)');
    subplot(2, 1, 2);
    imagesc(t, f, Sy); axis xy; caxis([top - 80, top]); colormap(jet); colorbar;
    title('Enhanced output (AI/ML adaptive noise cancellation)');
    ylabel('Frequency (kHz)'); xlabel('Time (s)');
end

function [S, t, f] = spec_db(x, fs)
    N = 512; R = 128;
    x = x(:);
    if numel(x) < N, x(N) = 0; end
    w = 0.5 - 0.5 * cos(2 * pi * (0:N-1)' / N);
    nf = floor((numel(x) - N) / R) + 1;
    idx = repmat((1:N)', 1, nf) + repmat((0:nf-1) * R, N, 1);
    X = fft(x(idx) .* repmat(w, 1, nf));
    S = 20 * log10(abs(X(1:N/2+1, :)) + 1e-9);
    t = ((0:nf-1) * R + N/2) / fs;
    f = (0:N/2) * fs / N / 1000;
end
