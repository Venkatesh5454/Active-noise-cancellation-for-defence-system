function flac_to_wav(in_dir, out_dir)
%FLAC_TO_WAV  Convert every .flac file in a folder to a 16-bit .wav file.
%
%   flac_to_wav('speech_clean')                    -> speech_clean_wav/
%   flac_to_wav('noise_general', 'D:\data\noise')
%
%   MATLAB's audioread reads FLAC directly, so this is only needed for tools
%   that accept WAV only. FLAC is lossless: the samples do not change.
    if nargin < 2, out_dir = [in_dir '_wav']; end
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end
    f = dir(fullfile(in_dir, '*.flac'));
    for k = 1:numel(f)
        [x, fs] = audioread(fullfile(in_dir, f(k).name), 'native');   % int16 samples
        [~, stem] = fileparts(f(k).name);
        audiowrite(fullfile(out_dir, [stem '.wav']), x, fs);
    end
    fprintf('%d files: %s -> %s\n', numel(f), in_dir, out_dir);
end
