#!/usr/bin/env python3
"""
Download the real recordings of the training data (clean speech and general
noise) again, straight from Microsoft's MS-SNSD repository on GitHub, and
check that every file is sample-for-sample identical to the audio the model
was trained on.

    python download_mssnsd_subset.py                  # speech + noise, saved as FLAC
    python download_mssnsd_subset.py --format wav     # save WAV files instead
    python download_mssnsd_subset.py --only noise     # only the 179 noise files
    python download_mssnsd_subset.py --limit 5        # quick test: 5 files per list

The file lists and checksums come from manifests/speech_clean.csv and
manifests/noise_general.csv. The files are taken from one fixed MS-SNSD
commit, so the result never changes. About 0.9 GB is downloaded. Files that
are already present and correct are skipped, so an interrupted run can simply
be started again.

The synthetic defence noise is not downloaded: it is made by the MATLAB code,
see make_noise_bank.m.
"""
import argparse, csv, hashlib, io, os, sys, time, urllib.request
from concurrent.futures import ThreadPoolExecutor
import soundfile as sf

COMMIT = "fe61c4ba0d9ac8dd7e23d719cc79f8947e1dc742"
BASE = "https://raw.githubusercontent.com/microsoft/MS-SNSD/" + COMMIT + "/"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LISTS = {"speech": ("speech_clean.csv", "speech_clean"),
         "noise": ("noise_general.csv", "noise_general")}


def pcm_sha256(x):
    return hashlib.sha256(x.astype("<i2").tobytes()).hexdigest()


def fetch(url, tries=5):
    for k in range(tries):
        try:
            with urllib.request.urlopen(url, timeout=120) as r:
                return r.read()
        except Exception:
            if k == tries - 1:
                raise
            time.sleep(2 ** (k + 1))


def get(row, out_dir, fmt):
    dst = os.path.join(out_dir, os.path.splitext(row["file"])[0] + "." + fmt)
    if os.path.exists(dst):
        x, _ = sf.read(dst, dtype="int16")
        if pcm_sha256(x) == row["pcm_sha256"]:
            return "already there"
    try:
        x, fs = sf.read(io.BytesIO(fetch(BASE + row["mssnsd_path"])), dtype="int16")
    except Exception as e:
        print(f"  failed: {row['mssnsd_path']} ({e})", flush=True)
        return "failed"
    if pcm_sha256(x) != row["pcm_sha256"]:
        print(f"  different audio: {row['mssnsd_path']}", flush=True)
        return "different"
    sf.write(dst, x, fs, format=fmt.upper(), subtype="PCM_16")
    return "downloaded"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=ROOT,
                    help="dataset folder that contains manifests/ (default: %(default)s)")
    ap.add_argument("--format", choices=("flac", "wav"), default="flac")
    ap.add_argument("--only", choices=sorted(LISTS))
    ap.add_argument("--jobs", type=int, default=8, help="parallel downloads")
    ap.add_argument("--limit", type=int, default=0, help="only the first N files of each list")
    a = ap.parse_args()
    problems = 0
    for key, (manifest, folder) in LISTS.items():
        if a.only and key != a.only:
            continue
        with open(os.path.join(a.root, "manifests", manifest), newline="") as f:
            rows = list(csv.DictReader(f))
        if a.limit:
            rows = rows[:a.limit]
        out = os.path.join(a.root, folder)
        os.makedirs(out, exist_ok=True)
        print(f"{folder}: {len(rows)} files -> {out}", flush=True)
        with ThreadPoolExecutor(a.jobs) as ex:
            res = list(ex.map(lambda r: get(r, out, a.format), rows))
        counts = {s: res.count(s) for s in sorted(set(res))}
        print(f"{folder}: {counts}", flush=True)
        problems += counts.get("failed", 0) + counts.get("different", 0)
    if problems:
        sys.exit(f"{problems} file(s) failed or differ; run the script again to retry.")
    print("All files are identical to the training data.")


if __name__ == "__main__":
    main()
