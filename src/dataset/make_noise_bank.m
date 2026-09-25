function make_noise_bank(out_dir, clips_per_type, clip_s, seed0)
%MAKE_NOISE_BANK  Write a bank of synthetic defence-noise clips (training data).
%
%   make_noise_bank('noisebank', 60, 12)   -> 11 types x 60 clips x 12 s
%
%   Used to train the GRU mask estimator (tools/train_dnn.py).  Seeds start
%   at seed0 (default 100000) so they never overlap with the evaluation
%   noise used in main_defence_anc.m.
    if nargin < 2, clips_per_type = 60; end
    if nargin < 3, clip_s = 12; end
    if nargin < 4, seed0 = 100000; end
    types = {'gunshot', 'machinegun', 'artillery', 'helicopter', 'missile', ...
             'drone', 'siren', 'vehicle', 'hum', 'wind', 'battlefield'};
    fs = 16000;
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end
    for i = 1:numel(types)
        for k = 1:clips_per_type
            n = defence_noise(types{i}, clip_s, fs, seed0 + 1000 * i + k);
            n = 0.99 * n / max(abs(n));
            audiowrite(fullfile(out_dir, sprintf('%s_%03d.wav', types{i}, k)), n, fs);
        end
        fprintf('%-12s %d clips\n', types{i}, clips_per_type);
    end
end
