#!/usr/bin/env python3
"""
Write noisy / clean training pairs to disk, using exactly the mixing recipe that
tools/train_dnn.py applies on the fly (so you can inspect, share or reuse the
training data, e.g. for another framework or for MATLAB experiments).

    python make_training_pairs.py --speech speech_clean \
        --noise noise_defence_synthetic noise_general --out pairs --n 500

Recipe for every pair (identical to train_dnn.make_batch):
  * a random 3 s stretch of clean speech; short utterances are joined with
    0.1-0.5 s pauses
  * 15 %  : room echo (synthetic impulse response, T60 = 0.15-0.6 s)
  * 30 %  : random microphone colouring (spectral tilt) on the speech
  *  5 %  : speech removed (noise-only example)
  * noise : 70 % from the defence-noise folder (first --noise folder),
            30 % from the other noise folders; 35 % chance of a second noise
            layered on top at 20-100 % strength; 30 % spectral tilt
  * SNR   : uniform -10 ... +20 dB (95 %) or +40 dB (5 %, almost clean)
  * level : mixture RMS = 0.05 x 10^(U(-20, +12) / 20)

The random numbers are drawn in the same order as in train_dnn.py. With
--seed 0 and the three folders of the training-data package, pair i is
exactly the i-th example that "train_dnn.py --seed 0" generated (apart from
the 16-bit rounding of the files). For the shipped model, pairs 1-128 were
used for the input normalisation statistics and pairs 129-16128 for training
steps 1-1000 (16 pairs per step), so --n 16128 rebuilds its whole training set.

Output: <out>/clean/pair_00001.flac, <out>/noisy/pair_00001.flac (16 kHz,
16-bit, lossless; --format wav for WAV files) and <out>/pairs.csv describing
every random choice. A pair whose peak would exceed 0.99 is scaled down
(clean and noisy by the same factor, so the SNR is unchanged); the factor
is in the file_gain column.
"""
import argparse, csv, glob, os, random
import numpy as np
import soundfile as sf

FS = 16000


def load_dir(d, min_len):
    files = sorted(glob.glob(os.path.join(d, "*.wav")) + glob.glob(os.path.join(d, "*.flac")))
    out = []
    for f in files:
        x, fs = sf.read(f, dtype="float32", always_2d=True)
        x = x.mean(1)
        if fs != FS or len(x) < min_len:
            continue
        x = x - x.mean()
        r = np.sqrt(np.mean(x ** 2))
        if r > 1e-6:
            out.append((os.path.basename(f), (x / r).astype(np.float32)))
    return out


def rand_seg(pool, L, allow_pause=False):
    """random L-sample segment; returns (signal, 'file@offset;...')"""
    name, x = pool[random.randrange(len(pool))]
    if len(x) >= L:
        s = random.randrange(len(x) - L + 1)
        return x[s:s + L].copy(), f"{name}@{s}"
    out = np.zeros(L, np.float32)
    used = []
    pos = 0 if not allow_pause else random.randrange(0, max(1, L - len(x)))
    while pos < L:
        n = min(len(x), L - pos)
        out[pos:pos + n] = x[:n]
        used.append(f"{name}@0+{pos}")
        pos += n + (random.randrange(1600, 8000) if allow_pause else 0)
        name, x = pool[random.randrange(len(pool))]
    return out, ";".join(used)


def rand_rir():
    t60 = random.uniform(0.15, 0.6)
    L = int(t60 * FS)
    t = np.arange(L) / FS
    h = np.random.randn(L) * np.exp(-6.9 * t / t60)
    h[0] = 1.0 / random.uniform(0.3, 1.0)
    return (h / np.sqrt(np.sum(h ** 2))).astype(np.float32), t60


def tilt(x):
    a = random.uniform(-0.5, 0.5)
    return (x - a * np.concatenate([[0.0], x[:-1]])).astype(np.float32), a


def make_pair(speech, noise_def, noise_gen, L):
    m = {}
    s, m["speech"] = rand_seg(speech, L, allow_pause=True)
    m["reverb_t60_s"] = 0.0
    if random.random() < 0.15:
        h, m["reverb_t60_s"] = rand_rir()
        s = np.convolve(s, h)[:L]
    m["speech_tilt"] = 0.0
    if random.random() < 0.3:
        s, m["speech_tilt"] = tilt(s)
    m["noise_only"] = 0
    if random.random() < 0.05:
        s[:] = 0.0
        m["noise_only"] = 1
    pool = noise_def if (random.random() < 0.7 or not noise_gen) else noise_gen
    n, m["noise1"] = rand_seg(pool, L)
    m["noise2"], m["noise2_gain"] = "", 0.0
    if random.random() < 0.35:
        pool2 = noise_gen if (noise_gen and random.random() < 0.6) else noise_def
        m["noise2_gain"] = random.uniform(0.2, 1.0)    # drawn before the segment, as in training
        n2, m["noise2"] = rand_seg(pool2, L)
        n = n + m["noise2_gain"] * n2
    m["noise_tilt"] = 0.0
    if random.random() < 0.3:
        n, m["noise_tilt"] = tilt(n)
    snr = random.uniform(-10, 20) if random.random() < 0.95 else 40.0
    m["snr_db"] = snr
    ps = np.mean(s ** 2)
    pn = np.mean(n ** 2) + 1e-12
    if ps > 0:
        n = n * np.sqrt(ps / (pn * 10 ** (snr / 10)))
    else:
        n = n * np.sqrt(0.0025 / pn)
    x = s + n
    lvl = random.uniform(-20, 12)
    m["level_db"] = lvl
    k = 0.05 * 10 ** (lvl / 20) / (np.sqrt(np.mean(x ** 2)) + 1e-9)
    return s * k, x * k, m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speech", required=True)
    ap.add_argument("--noise", nargs="+", required=True,
                    help="first folder = defence noise, others = general noise")
    ap.add_argument("--out", required=True)
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--format", choices=("flac", "wav"), default="flac")
    a = ap.parse_args()
    random.seed(a.seed); np.random.seed(a.seed)
    L = int(a.seconds * FS)
    speech = load_dir(a.speech, 4000)
    noise_def = load_dir(a.noise[0], 8000)
    noise_gen = [x for d in a.noise[1:] for x in load_dir(d, 8000)]
    print(f"speech {len(speech)} files, defence noise {len(noise_def)}, other noise {len(noise_gen)}")
    for sub in ("clean", "noisy"):
        os.makedirs(os.path.join(a.out, sub), exist_ok=True)
    rows = []
    for i in range(1, a.n + 1):
        s, x, m = make_pair(speech, noise_def, noise_gen, L)
        peak = max(np.max(np.abs(x)), np.max(np.abs(s)))
        m["file_gain"] = 1.0
        if peak > 0.99:                       # keep 16-bit files unclipped
            m["file_gain"] = float(0.99 / peak)
            s, x = s * m["file_gain"], x * m["file_gain"]
        name = f"pair_{i:05d}.{a.format}"
        sf.write(os.path.join(a.out, "clean", name), s, FS, subtype="PCM_16")
        sf.write(os.path.join(a.out, "noisy", name), x, FS, subtype="PCM_16")
        rows.append({"pair": name, **{k: round(v, 4) if isinstance(v, float) else v
                                      for k, v in m.items()}})
    with open(os.path.join(a.out, "pairs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"{a.n} pairs written to {a.out}")


if __name__ == "__main__":
    main()
