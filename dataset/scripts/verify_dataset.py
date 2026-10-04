#!/usr/bin/env python3
"""
Check the unzipped training-data package: every file listed in
manifests/checksums_sha256.txt must be present and unchanged.

    python verify_dataset.py              (run from anywhere)

The result is reported per folder, and any zip file that is missing or
damaged is named, so you know which one to download or unzip again.
Works on Windows, Linux and macOS (Python 3, no extra packages).
"""
import collections, csv, hashlib, os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRST_ZIP = "zip 01 (docs, scripts, examples)"


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def main():
    listing = os.path.join(ROOT, "manifests", "checksums_sha256.txt")
    if not os.path.exists(listing):
        sys.exit(f"{listing} not found: unzip the first zip (docs, scripts, examples) first.")
    zip_of = {}
    contents = os.path.join(ROOT, "manifests", "zip_contents.csv")
    if os.path.exists(contents):
        with open(contents, newline="", encoding="utf-8") as f:
            zip_of = {r["path"]: r["zip"] for r in csv.DictReader(f)}
    stats = collections.OrderedDict()
    bad_zips = collections.OrderedDict()
    with open(listing, encoding="utf-8") as f:
        for line in f:
            digest, rel = line.rstrip("\n").split("  ", 1)
            top = rel.split("/")[0] if "/" in rel else "(top level)"
            s = stats.setdefault(top, collections.Counter())
            path = os.path.join(ROOT, *rel.split("/"))
            if not os.path.exists(path):
                state = "missing"
            elif sha256(path) != digest:
                state = "damaged"
                print("  damaged:", rel)
            else:
                state = "ok"
            s[state] += 1
            if state != "ok":
                bad_zips.setdefault(zip_of.get(rel, FIRST_ZIP), collections.Counter())[state] += 1
    for top, s in stats.items():
        total = sum(s.values())
        if s["ok"] == total:
            state = "complete"
        elif s["missing"] == total:
            state = "not unzipped yet"
        else:
            state = f"{s['missing']} missing, {s['damaged']} damaged"
        print(f"{top:26s} {s['ok']:5d} / {total:5d} files OK   {state}")
    if not bad_zips:
        print("Everything is complete and intact.")
        return
    print("\nDownload / unzip these zip files (again):")
    for z, c in sorted(bad_zips.items()):
        print(f"  {z}   ({', '.join(f'{n} {k}' for k, n in c.items())})")
    sys.exit(1)


if __name__ == "__main__":
    main()
