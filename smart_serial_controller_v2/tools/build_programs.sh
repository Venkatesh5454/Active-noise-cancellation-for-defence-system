#!/bin/sh
# -----------------------------------------------------------------------------
# build_programs.sh - assemble every serial engine program
#     sh tools/build_programs.sh
# Reads programs/*.se and writes programs/<name>.hex, programs/<name>.lst
# and one C header sw/se_programs.h with all the programs.
# Run it again after adding or changing a program.
# -----------------------------------------------------------------------------
cd "$(dirname "$0")/.." || exit 1
set -- programs/*.se
if [ ! -e "$1" ]; then
    echo "build_programs: no programs/*.se found" >&2
    exit 1
fi
exec python3 tools/se_asm.py -o sw/se_programs.h --outdir programs "$@"
