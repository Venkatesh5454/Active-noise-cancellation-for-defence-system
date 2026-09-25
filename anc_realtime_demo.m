%% ANC_REALTIME_DEMO  Live microphone -> headphone noise cancellation.
%
%  Streams audio in 8 ms blocks through the same causal engine that
%  main_defence_anc.m uses (anc_process_frame), i.e. exactly what runs on an
%  embedded board.  Needs MATLAB's Audio Toolbox for the sound-card I/O
%  (audioDeviceReader / audioDeviceWriter).  Without it the script streams
%  a file block-by-block instead, which demonstrates the same real-time
%  loop and reports the per-block compute time.
%
%  Wear headphones (to avoid acoustic feedback) and press Ctrl+C to stop.

clear; clc;
root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root, 'src')));

duration_s = 30;                    % length of the live session
p = anc_params();
p.model = dnn_load();
p.use_dnn = ~isempty(p.model);
st = anc_init(p);
R = st.R;                           % 128 samples = 8 ms @ 16 kHz

if exist('audioDeviceReader', 'class') == 8 || exist('audioDeviceReader', 'file') == 2
    mic = audioDeviceReader('SampleRate', p.fs, 'SamplesPerFrame', R, 'NumChannels', 1);
    spk = audioDeviceWriter('SampleRate', p.fs);
    fprintf('Live ANC running for %d s (latency %.0f ms) ...\n', duration_s, 1000 * st.N / p.fs);
    t_proc = 0; nblk = round(duration_s * p.fs / R);
    for b = 1:nblk
        x = mic();
        t0 = tic;
        [y, st] = anc_process_frame(double(x(:, 1)), st);
        t_proc = t_proc + toc(t0);
        spk(y);
    end
    release(mic); release(spk);
else
    fprintf('Audio Toolbox not found - streaming a file block-by-block instead.\n');
    [x, fs] = read_audio(fullfile(root, 'audio', 'noisy', 'defence_sound.mp3'));
    x = resample_k(mean(x, 2), p.fs, fs);
    nblk = floor(numel(x) / R);
    y = zeros(nblk * R, 1);
    t_proc = 0;
    for b = 1:nblk
        idx = (b - 1) * R + (1:R);
        t0 = tic;
        [y(idx), st] = anc_process_frame(x(idx), st);
        t_proc = t_proc + toc(t0);
    end
    audiowrite(fullfile(root, 'results', 'realtime_stream_output.wav'), ...
               y / max(abs(y)) * 0.95, p.fs);
end
fprintf('Average compute time per 8 ms block: %.3f ms (real-time factor %.3f)\n', ...
        1000 * t_proc / nblk, t_proc / (nblk * R / p.fs));
