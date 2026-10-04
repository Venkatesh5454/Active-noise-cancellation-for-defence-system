#!/usr/bin/env python3
"""
Convert the FLAC files of one or more folders to 16-bit WAV. FLAC is lossless,
so the WAV files contain exactly the same samples. Only needed for tools that
cannot read FLAC (MATLAB, Octave, Python soundfile and Audacity all can).

    python flac_to_wav.py ../speech_clean ../noise_general
        -> ../speech_clean_wav/  ../noise_general_wav/
"""
import argparse, glob, os
import soundfile as sf

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("folders", nargs="+")
for d in ap.parse_args().folders:
    d = os.path.normpath(d)
    out = d + "_wav"
    os.makedirs(out, exist_ok=True)
    files = sorted(glob.glob(os.path.join(d, "*.flac")))
    for f in files:
        x, fs = sf.read(f, dtype="int16")
        sf.write(os.path.join(out, os.path.basename(f)[:-5] + ".wav"), x, fs, subtype="PCM_16")
    print(f"{len(files)} files: {d} -> {out}")
