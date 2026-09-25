%% MAIN_DEFENCE_ANC  AI/ML-enabled adaptive noise cancellation for defence audio
%  SIH 2026 - Problem Statement 26052 (DRDO)
%
%  Removes stationary (engine / rotor hum), non-stationary (missile whine,
%  helicopter, sirens) and impulsive (gunshots, machine-gun fire,
%  explosions) noise from speech, plays the result and reports SNR, STOI
%  and PESQ.
%
%  Just press F5 / type  main_defence_anc  in the MATLAB command window.
%  No toolboxes are required (tested with MATLAB-compatible GNU Octave 8).
%
%  PART 1  cleans your recordings (audio/noisy/*), saves them to results/,
%          plays "before" and "after" and plots the spectrograms.
%          A real recording has no clean reference, so SNR is *estimated*
%          blindly there (STOI / PESQ need the clean speech by definition).
%  PART 2  measures true SNR / STOI / PESQ: known clean speech is mixed with
%          defence noise at a known SNR, cleaned, and compared with the clean
%          original (the evaluation protocol of the problem statement).

clear; close all; clc;
root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root, 'src')));

%% ------------------------------- settings --------------------------------
input_files = { fullfile(root, 'audio', 'noisy', 'defence_sound.mp3'), ...
                fullfile(root, 'audio', 'noisy', 'defence_soundMG1.mpeg') };
clean_reference = '';     % optional: the clean speech contained in input_files{1}
                          % (if you created the noisy file yourself) -> exact metrics
play_audio  = true;       % listen to before / after
make_plots  = true;
out_dir     = fullfile(root, 'results');
test_snrs   = [0 5];      % PART 2 input SNRs (dB)
test_noises = {'battlefield', 'machinegun', 'gunshot', 'helicopter', 'missile'};

%% ------------------------------ the filter --------------------------------
p = anc_params();                  % all tunable parameters (see anc_params.m)
p.model = dnn_load();              % trained GRU mask estimator (models/)
if isempty(p.model)
    fprintf('Model file not found -> using the statistical OM-LSA filter only.\n');
    p.use_dnn = false;
end
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

%% ============== PART 1 : clean the supplied recordings =====================
for i = 1:numel(input_files)
    [x, fs] = read_audio(input_files{i});
    [y, fs_y, info] = defence_anc(x, fs, p);            % <-- noise cancellation

    [~, name, ext] = fileparts(input_files{i});
    out_file = fullfile(out_dir, [name '_enhanced.wav']);
    audiowrite(out_file, y, fs_y);

    xm = resample_k(mean(x, 2), fs_y, fs);             % noisy input @16 kHz
    xm = xm(1:numel(y));
    b = blind_snr(xm, y, fs_y);

    fprintf('\n=================================================================\n');
    fprintf(' %s  (%.1f s, %s, %d Hz)\n', [name ext], ...
            size(x, 1) / fs, info.mode, fs);
    fprintf('=================================================================\n');
    fprintf(' latency %.0f ms | real-time factor %.3f | saved: %s\n', ...
            info.latency_ms, info.rtf, out_file);
    fprintf(' Estimated SNR (no clean reference): input %6.1f dB  ->  output %6.1f dB\n', ...
            b.snr_in, b.snr_out);
    fprintf(' Background noise reduction in speech pauses: %.1f dB\n', b.noise_reduction);

    if i == 1 && ~isempty(clean_reference)
        [c, fc] = read_audio(clean_reference);
        c = resample_k(mean(c, 2), fs_y, fc);
        n = min([numel(c) numel(y)]);
        report_metrics('Your recording (with clean reference)', c(1:n), xm(1:n), y(1:n), fs_y);
    end

    if make_plots
        try
            plot_spectrograms(xm, y, fs_y, [name ext]);
            drawnow;
        catch
            fprintf(' (plotting not available on this system)\n');
        end
    end
    if play_audio
        play_audio_file(xm, fs_y, 'ORIGINAL (noisy)');
        play_audio_file(y, fs_y, 'ENHANCED');
    end
end

%% ========= PART 2 : SNR / STOI / PESQ with known clean speech =============
% clean test speech: CMU ARCTIC utterances (male + female, not used in training)
cf = dir(fullfile(root, 'audio', 'clean', '*.wav'));
spk = {'aew', 'axb'};
fprintf('\n\n######## Objective evaluation (clean speech + defence noise) ########\n');
fprintf('%-12s %5s | %-22s | %-24s | %-22s | %-22s\n', 'noise', 'SNRin', ...
        'SNR (dB) noisy->enh', 'STOI noisy->enh', 'PESQ-WB noisy->enh', 'PESQ-NB noisy->enh');
fprintf('%s\n', repmat('-', 1, 124));
T = [];
for iz = 1:numel(test_noises)
    for s_in = test_snrs
        for k = 1:numel(spk)
            files = cf(~cellfun(@isempty, strfind({cf.name}, spk{k})));
            clean = [];
            for j = 1:numel(files)
                [c, fc] = audioread(fullfile(files(j).folder, files(j).name));
                clean = [clean; zeros(round(0.3 * fc), 1); resample_k(c(:, 1), 16000, fc)]; %#ok<AGROW>
            end
            clean = clean / max(abs(clean)) * 0.5;
            noise = defence_noise(test_noises{iz}, numel(clean) / 16000, 16000, 7 + iz + 10 * k);
            noisy = mix_at_snr(clean, noise, s_in);
            enh = defence_anc(noisy, 16000, p);
            r = [evaluate_pair(clean, noisy, 16000), evaluate_pair(clean, enh, 16000)];
            T = [T; iz, s_in, k, r]; %#ok<AGROW>
        end
        rows = T(T(:, 1) == iz & T(:, 2) == s_in, 4:end);
        mm = mean(rows, 1);          % [snr stoi pesqwb pesqnb] noisy, then enhanced
        fprintf('%-12s %+5d | %8.2f -> %8.2f  | %8.3f -> %8.3f     | %7.2f -> %7.2f     | %7.2f -> %7.2f\n', ...
                test_noises{iz}, s_in, mm(1), mm(5), mm(2), mm(6), mm(3), mm(7), mm(4), mm(8));
    end
end
fprintf('%s\n', repmat('-', 1, 124));
for s_in = test_snrs
    mm = mean(T(T(:, 2) == s_in, 4:end), 1);
    fprintf('%-12s %+5d | %8.2f -> %8.2f  | %8.3f -> %8.3f     | %7.2f -> %7.2f     | %7.2f -> %7.2f\n', ...
            'AVERAGE', s_in, mm(1), mm(5), mm(2), mm(6), mm(3), mm(7), mm(4), mm(8));
end
fprintf('\nTargets (problem statement): SNR > 15 dB, STOI > 0.85, PESQ > 2.5\n');
