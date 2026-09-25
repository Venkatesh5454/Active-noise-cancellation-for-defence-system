function [x, fs] = read_audio(file)
%READ_AUDIO  audioread() that also accepts .mpeg / .mpg files.
%   MATLAB chooses the decoder from the file extension, so an MP3 stream
%   saved as .mpeg (like defence_soundMG1.mpeg) is copied to a temporary
%   .mp3 file first.  Returns all channels as double.
    [~, ~, ext] = fileparts(file);
    if any(strcmpi(ext, {'.mpeg', '.mpg', '.mpga', '.mp2'}))
        tmp = [tempname '.mp3'];
        copyfile(file, tmp);
        try
            [x, fs] = audioread(tmp);
        catch err
            delete(tmp);
            rethrow(err);
        end
        delete(tmp);
    else
        [x, fs] = audioread(file);
    end
    x = double(x);
end
