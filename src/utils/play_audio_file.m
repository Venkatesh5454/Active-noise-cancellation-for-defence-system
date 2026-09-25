function play_audio_file(v, fs, label)
%PLAY_AUDIO_FILE  Play a signal (normalised) and wait until it has finished.
    fprintf(' Playing %s ...\n', label);
    try
        sound(v / max(abs(v)) * 0.9, fs);
        pause(numel(v) / fs + 0.5);
    catch
        fprintf('   (no audio device available - listen to the WAV files in results/)\n');
    end
end
