function make_dataset(clean_dir, out_dir, n_pairs, snr_range, noise_types)
%MAKE_DATASET  Scalable noisy / clean speech pair generator.
%
%   make_dataset('audio/clean', 'dataset', 200)
%   make_dataset(clean_dir, out_dir, n_pairs, [-10 20], {'machinegun','gunshot'})
%
%   For every pair a random clean utterance is mixed with a random defence
%   noise (defence_noise.m) at a random SNR drawn from snr_range (dB), with
%   augmentation: random level, reverberation (synthetic room impulse
%   response) and occasional clipping.  Writes
%       out_dir/clean/pair_00001.wav   out_dir/noisy/pair_00001.wav
%       out_dir/manifest.csv           (file, speech, noise, snr, reverb, clip)
    if nargin < 3, n_pairs = 100; end
    if nargin < 4, snr_range = [-10 20]; end
    if nargin < 5
        noise_types = {'gunshot', 'machinegun', 'artillery', 'helicopter', ...
            'missile', 'drone', 'siren', 'vehicle', 'hum', 'wind', 'battlefield'};
    end
    fs = 16000;
    files = [dir(fullfile(clean_dir, '*.wav')); dir(fullfile(clean_dir, '*.flac'))];
    if isempty(files), error('make_dataset: no .wav or .flac files in %s', clean_dir); end
    if ~exist(fullfile(out_dir, 'clean'), 'dir'), mkdir(fullfile(out_dir, 'clean')); end
    if ~exist(fullfile(out_dir, 'noisy'), 'dir'), mkdir(fullfile(out_dir, 'noisy')); end
    fid = fopen(fullfile(out_dir, 'manifest.csv'), 'w');
    fprintf(fid, 'file,speech,noise,snr_db,reverb_t60,clipped\n');
    for i = 1:n_pairs
        f = files(randi(numel(files)));
        [s, fs0] = audioread(fullfile(clean_dir, f.name));
        s = resample_k(mean(s, 2), fs, fs0);
        s = s / max(abs(s) + eps) * 0.5;
        t60 = 0;
        if rand < 0.3                                   % reverberation
            t60 = 0.2 + 0.5 * rand;
            L = round(t60 * fs);
            h = randn(L, 1) .* exp(-6.9 * (0:L-1)' / L);
            h(1) = 3;
            s = filter(h / norm(h), 1, s);
        end
        nt = noise_types{randi(numel(noise_types))};
        snr = snr_range(1) + diff(snr_range) * rand;
        n = defence_noise(nt, numel(s) / fs, fs, 500000 + i);
        noisy = mix_at_snr(s, n, snr);
        g = 10^((-20 + 14 * rand) / 20) / max(abs(noisy));   % random level
        noisy = noisy * g;  s = s * g;
        clipped = rand < 0.05;
        if clipped
            noisy = max(min(noisy, 0.7 * max(abs(noisy))), -0.7 * max(abs(noisy)));
        end
        name = sprintf('pair_%05d.wav', i);
        audiowrite(fullfile(out_dir, 'clean', name), s, fs);
        audiowrite(fullfile(out_dir, 'noisy', name), noisy, fs);
        fprintf(fid, '%s,%s,%s,%.2f,%.2f,%d\n', name, f.name, nt, snr, t60, clipped);
    end
    fclose(fid);
    fprintf('%d noisy/clean pairs written to %s\n', n_pairs, out_dir);
end
