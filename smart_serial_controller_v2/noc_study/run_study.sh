#!/usr/bin/env bash
# =============================================================================
# run_study.sh  -  NoC scalability study (slide 24, measurement 6)
# -----------------------------------------------------------------------------
# Compares a shared bus, a crossbar and a 2-D mesh NoC for N = 4, 8 and 16
# nodes:
#   1. simulation (Icarus Verilog): noc_study/tb_study.v +sweep for every
#      fabric and N -> latency and throughput at 8 offered loads
#   2. synthesis (yosys synth_xilinx, Zynq-7000 cells): LUTs, FFs and the
#      longest LUT chain for every fabric and N  (ESTIMATES; run
#      noc_study/vivado_study.tcl in Vivado for real LUTs and Fmax)
#   3. writes the tables to noc_study/results.md
#
# Usage (Linux or WSL, from any folder):
#     bash noc_study/run_study.sh            # everything (about 10 minutes)
#     bash noc_study/run_study.sh sim        # only the simulations
#     bash noc_study/run_study.sh synth      # only yosys
#     bash noc_study/run_study.sh report     # only rebuild results.md
#     JOBS=4 bash noc_study/run_study.sh     # run 4 simulations at a time
#
# Needs: iverilog + vvp (version 11 or newer), yosys (0.20 or newer), awk.
# Outputs: build/study_<fabric>_n<N>.log       one simulation log each
#          build/study_synth_<fabric>_n<N>.txt one yosys report each
#          build/study_sim.txt, build/study_synth.txt   collected results
#          noc_study/results.md                         the tables
# If noc_study/vivado_reports/summary.txt exists (written by
# vivado_study.tcl) it is copied into results.md as well.
# =============================================================================
set -u
cd "$(dirname "$0")/.." || exit 1
mkdir -p build

WHAT="${1:-all}"
JOBS="${JOBS:-1}"
FABRICS="bus xbar mesh"
SIZES="4 8 16"
SIM_RTL="noc_study/rtl/study_bus.v noc_study/rtl/study_xbar.v noc_study/rtl/study_mesh.v
         rtl/noc/noc_mesh.v rtl/noc/noc_router.v"
FAILED=""

fabric_id() { case "$1" in bus) echo 0 ;; xbar) echo 1 ;; mesh) echo 2 ;; esac; }

fabric_files() {
    case "$1" in
        bus)  echo "noc_study/rtl/study_bus.v" ;;
        xbar) echo "noc_study/rtl/study_xbar.v" ;;
        mesh) echo "noc_study/rtl/study_mesh.v rtl/noc/noc_mesh.v rtl/noc/noc_router.v" ;;
    esac
}

# ---------------------------------------------------------------------------
# 1. simulations
# ---------------------------------------------------------------------------
sim_one() {     # sim_one <fabric> <N>
    local f=$1 n=$2 tag="study_$1_n$2" t0 t1
    t0=$(date +%s)
    # shellcheck disable=SC2086
    if ! iverilog -g2005 -Wall -P tb_study.FABRIC="$(fabric_id "$f")" -P tb_study.N="$n" \
            -o "build/$tag.vvp" noc_study/tb_study.v $SIM_RTL; then
        echo "  $f N=$n: COMPILE FAILED"
        return 1
    fi
    vvp -n "build/$tag.vvp" +sweep > "build/$tag.log" 2>&1
    t1=$(date +%s)
    printf "  %-4s N=%-2s : %s (%s s)\n" "$f" "$n" \
        "$(grep -E 'CHECKS (PASSED|FAILED)' "build/$tag.log" | tail -n 1)" "$((t1 - t0))"
    grep -q "ALL .* CHECKS PASSED" "build/$tag.log"
}

run_sims() {
    echo "### simulations (tb_study +sweep, $JOBS at a time)"
    local f n
    # the 16-node mesh is by far the slowest, so it starts first
    for n in 16 8 4; do
        for f in mesh xbar bus; do
            sim_one "$f" "$n" &
            while [ "$(jobs -rp | wc -l)" -ge "$JOBS" ]; do sleep 1; done
        done
    done
    wait
    : > build/study_sim.txt
    for n in $SIZES; do
        for f in $FABRICS; do
            if grep -q "ALL .* CHECKS PASSED" "build/study_${f}_n${n}.log" 2>/dev/null; then
                grep '^RESULT' "build/study_${f}_n${n}.log" >> build/study_sim.txt
            else
                FAILED="$FAILED sim_${f}_n${n}"
            fi
        done
    done
}

# ---------------------------------------------------------------------------
# 2. yosys synthesis (one at a time: the big ones need a few hundred MB)
# ---------------------------------------------------------------------------
synth_one() {   # synth_one <fabric> <N>
    local f=$1 n=$2 tag="study_synth_$1_n$2" t0 t1
    t0=$(date +%s)
    # -noiopad: no I/O buffers, like Vivado's out-of-context synthesis (a
    # 16-node fabric has far more ports than the chip has pins)
    if ! yosys -q -p "read_verilog $(fabric_files "$f"); chparam -set N $n study_$f;
            synth_xilinx -top study_$f -flatten -noiopad;
            tee -q -o build/$tag.txt stat;
            tee -q -a build/$tag.txt ltp -noff t:LUT* t:MUXF* t:CARRY4 w:*" \
            > "build/$tag.log" 2>&1; then
        echo "  $f N=$n: YOSYS FAILED (see build/$tag.log)"
        FAILED="$FAILED synth_${f}_n${n}"
        return 1
    fi
    t1=$(date +%s)
    # LUT = logic LUTs + LUTs used as distributed RAM (RAM32M = 4 LUTs).
    # yosys INV cells are not counted: they are one inverter per flip-flop
    # for the active-low reset, which Vivado merges into one.
    awk -v f="$f" -v n="$n" '
        $1 ~ /^LUT[1-6]$/                               { lut += $2 }
        $1 ~ /^(RAM32M|RAM64M|RAM128X1D|RAM256X1S)$/    { mem += 4 * $2 }
        $1 ~ /^(RAM32X1D|RAM64X1D)$/                    { mem += 2 * $2 }
        $1 ~ /^(RAM32X1S|RAM64X1S|SRL16E|SRLC32E)$/     { mem += $2 }
        $1 ~ /^FD[CPRS]E/                               { ff += $2 }
        $1 ~ /^MUXF[78]$/                               { muxf += $2 }
        $1 == "CARRY4"                                  { carry += $2 }
        /Longest topological path/ {
            if (match($0, /length=[0-9]+/)) lv = substr($0, RSTART + 7, RLENGTH - 7)
        }
        END {
            printf "SYNTH fabric=%s n=%s lut=%d lut_logic=%d lut_mem=%d ff=%d muxf=%d carry=%d levels=%s\n",
                   f, n, lut + mem, lut, mem, ff, muxf, carry, lv
        }' "build/$tag.txt" >> build/study_synth.txt
    printf "  %-4s N=%-2s : %s (%s s)\n" "$f" "$n" \
        "$(tail -n 1 build/study_synth.txt | cut -d' ' -f4-)" "$((t1 - t0))"
}

run_synth() {
    echo "### yosys synth_xilinx (estimates)"
    local f n
    : > build/study_synth.txt
    for n in $SIZES; do
        for f in $FABRICS; do
            synth_one "$f" "$n"
        done
    done
}

# ---------------------------------------------------------------------------
# 3. results.md
# ---------------------------------------------------------------------------
make_report() {
    echo "### writing noc_study/results.md"
    local out=noc_study/results.md
    local iv ys
    touch build/study_sim.txt build/study_synth.txt
    iv=$(iverilog -V 2>/dev/null | head -n 1 | sed 's/ (.*//')
    ys=$(yosys -V 2>/dev/null | sed 's/ (.*//')
    {
        echo "# NoC scalability study: results"
        echo
        echo "Generated by \`noc_study/run_study.sh\` on $(date '+%Y-%m-%d') with $iv and $ys."
        echo "How the numbers are made and what they mean: see [README.md](README.md)."
        echo "All three fabrics have the same node interface (34-bit flits, credits,"
        echo "DEPTH = 4 buffers) and get exactly the same random traffic."
        echo
        awk '
        function kv(line,    i, n, a, p) {
            delete V
            n = split(line, a, " ")
            for (i = 2; i <= n; i++) {
                p = index(a[i], "=")
                if (p > 0) V[substr(a[i], 1, p - 1)] = substr(a[i], p + 1)
            }
        }
        FILENAME == ARGV[1] && /^SYNTH/ {
            kv($0); k = V["fabric"] SUBSEP V["n"]
            LUT[k] = V["lut"]; LMEM[k] = V["lut_mem"]; FF[k] = V["ff"]; LV[k] = V["levels"]
            haveS = 1
        }
        FILENAME == ARGV[2] && /^RESULT/ {
            kv($0); k = V["fabric"] SUBSEP V["n"] SUBSEP V["rate"]
            ACC[k] = V["accepted"]; TOT[k] = V["total"]; LAT[k] = V["lat_avg"]
            LMAX[k] = V["lat_max"]; SAT[k] = V["sat"]; NET[k] = V["net_avg"]
            if (!(V["rate"] in seen)) { seen[V["rate"]] = 1; R[++nr] = V["rate"] }
            haveR = 1
        }
        function lat(f, n, r,    k) {
            k = f SUBSEP n SUBSEP r
            if (!(k in LAT)) return "-"
            if (SAT[k] == 1) return "sat"
            return sprintf("%.1f", LAT[k])
        }
        END {
            split("bus xbar mesh", F, " ")
            split("4 8 16", NS, " ")
            NAME["bus"] = "bus"; NAME["xbar"] = "crossbar"; NAME["mesh"] = "mesh"
            SHAPE["mesh" SUBSEP 4] = " (2x2)"; SHAPE["mesh" SUBSEP 8] = " (4x2)"; SHAPE["mesh" SUBSEP 16] = " (4x4)"

            print "## 1. Area (yosys estimate, not Vivado)"
            print ""
            if (!haveS) { print "(no synthesis results: run `bash noc_study/run_study.sh synth`)"; print "" }
            else {
                print "| N | fabric | LUTs | of which LUTRAM | FFs | LUTs per node | logic levels |"
                print "|---:|---|---:|---:|---:|---:|---:|"
                for (i = 1; i <= 3; i++) for (j = 1; j <= 3; j++) {
                    n = NS[i]; f = F[j]; k = f SUBSEP n
                    if (k in LUT)
                        printf "| %s | %s%s | %d | %d | %d | %.0f | %s |\n", n, NAME[f], SHAPE[f SUBSEP n], \
                               LUT[k], LMEM[k], FF[k], LUT[k] / n, LV[k]
                }
                print ""
                print "- yosys `synth_xilinx -flatten -noiopad` (no I/O buffers, like Vivado out-of-context)."
                print "  LUTs = logic LUTs + LUTs used as distributed RAM for the FIFOs (RAM32M = 4 LUTs)."
                print "  Vivado usually needs fewer LUTs (better wide-mux mapping); use the ratios, not the exact values."
                print "- logic levels = longest chain of LUT / MUXF7 / MUXF8 / CARRY4 cells between two"
                print "  flip-flops (yosys `ltp`), a rough stand-in for the clock speed: more levels, lower Fmax."
                print "  The distributed-RAM read at the start of the path is not counted."
                print ""
            }

            if (!haveR) { print "(no simulation results: run `bash noc_study/run_study.sh sim`)"; exit }
            lo = R[1]; hi = R[nr]
            print "## 2. Latency at low load (offered " lo " flits/node/clock)"
            print ""
            print "Packet latency in clocks, from the clock the packet is made to the clock its"
            print "TAIL flit reaches the destination (4-flit packets, average / worst)."
            print ""
            print "| N | bus | crossbar | mesh |"
            print "|---:|---:|---:|---:|"
            for (i = 1; i <= 3; i++) {
                n = NS[i]; line = "| " n " |"
                for (j = 1; j <= 3; j++) {
                    k = F[j] SUBSEP n SUBSEP lo
                    line = line " " ((k in LAT) ? sprintf("%.1f / %d", LAT[k], LMAX[k]) : "-") " |"
                }
                print line
            }
            print ""
            print "## 3. Saturation throughput (offered " hi " flits/node/clock: every source always busy)"
            print ""
            print "Accepted throughput: flits per node per clock (and for the whole fabric, flits per clock)."
            print ""
            print "| N | bus | crossbar | mesh | total flits/clock per 1000 LUTs (bus / xbar / mesh) |"
            print "|---:|---:|---:|---:|---|"
            for (i = 1; i <= 3; i++) {
                n = NS[i]; line = "| " n " |"; eff = ""
                for (j = 1; j <= 3; j++) {
                    k = F[j] SUBSEP n SUBSEP hi
                    line = line " " ((k in ACC) ? sprintf("%.3f (%.2f)", ACC[k], TOT[k]) : "-") " |"
                    ks = F[j] SUBSEP n
                    e = ((k in TOT) && (ks in LUT) && LUT[ks] > 0) ? sprintf("%.2f", 1000 * TOT[k] / LUT[ks]) : "-"
                    eff = eff (j > 1 ? " / " : "") e
                }
                print line " " eff " |"
            }
            print ""
            print "## 4. Full sweep"
            print ""
            print "For every offered load: accepted throughput (flits/node/clock) and average packet"
            print "latency (clocks). **sat** = the fabric cannot carry this load (the source queues"
            print "fill up), so the latency grows without limit and is not shown."
            for (i = 1; i <= 3; i++) {
                n = NS[i]
                print ""
                print "### N = " n
                print ""
                print "| offered | bus accepted | bus latency | xbar accepted | xbar latency | mesh accepted | mesh latency |"
                print "|---:|---:|---:|---:|---:|---:|---:|"
                for (r = 1; r <= nr; r++) {
                    line = "| " R[r] " |"
                    for (j = 1; j <= 3; j++) {
                        k = F[j] SUBSEP n SUBSEP R[r]
                        line = line " " ((k in ACC) ? ACC[k] : "-") " | " lat(F[j], n, R[r]) " |"
                    }
                    print line
                }
            }
            print ""
        }' build/study_synth.txt build/study_sim.txt

        echo "## 5. Vivado (real LUTs and Fmax)"
        echo
        if [ -f noc_study/vivado_reports/summary.txt ]; then
            echo "From \`noc_study/vivado_reports/summary.txt\` (written by \`vivado_study.tcl\`):"
            echo
            cat noc_study/vivado_reports/summary.txt
        else
            echo "Not run yet. In a shell with Vivado on the PATH, from the project folder:"
            echo
            echo '    vivado -mode batch -source noc_study/vivado_study.tcl'
            echo
            echo "(Windows: double-click \`windows\\5_noc_study_vivado.bat\`.)  It writes"
            echo "\`noc_study/vivado_reports/summary.txt\`; then run"
            echo "\`bash noc_study/run_study.sh report\` to copy it here."
        fi
        echo
        echo "## Raw result lines"
        echo
        echo "The simulation lines (\`build/study_sim.txt\`) and the yosys lines (\`build/study_synth.txt\`):"
        echo
        echo '```'
        cat build/study_synth.txt build/study_sim.txt
        echo '```'
    } > "$out"
}

# ---------------------------------------------------------------------------
case "$WHAT" in
    all)    run_sims; run_synth; make_report ;;
    sim)    run_sims; make_report ;;
    synth)  run_synth; make_report ;;
    report) make_report ;;
    *)      echo "usage: bash noc_study/run_study.sh [all|sim|synth|report]"; exit 2 ;;
esac

echo
if [ -n "$FAILED" ]; then
    echo "STUDY FAILED:$FAILED"
    exit 1
fi
echo "STUDY DONE: see noc_study/results.md"
