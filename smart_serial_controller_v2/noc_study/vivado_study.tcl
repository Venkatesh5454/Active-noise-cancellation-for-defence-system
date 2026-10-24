# =============================================================================
# vivado_study.tcl  -  NoC scalability study in Vivado: real LUTs and Fmax
# -----------------------------------------------------------------------------
# Synthesises the three fabrics (study_bus, study_xbar, study_mesh) for
# N = 4, 8 and 16 nodes, out of context (no I/O pins), for the ZedBoard part
# xc7z020clg484-1 with a 100 MHz clock constraint on clk.  No project is
# needed: it runs in Vivado's non-project batch mode.
#
#   Linux / WSL:  source <Vivado install>/settings64.sh
#                 cd smart_serial_controller_v2
#                 vivado -mode batch -nojournal -source noc_study/vivado_study.tcl
#   Windows:      double-click windows\5_noc_study_vivado.bat
#
# Options (after -tclargs, any order):
#   impl        also run opt_design, place_design and route_design and take
#               the numbers from the routed design (slower, more accurate)
#   quick       only N = 4 and 8
#   period=X    clock period in ns (default 10.0 = 100 MHz)
#   e.g.  vivado -mode batch -source noc_study/vivado_study.tcl -tclargs impl
#
# Results, in noc_study/vivado_reports/:
#   util_<fabric>_n<N>.txt     report_utilization
#   timing_<fabric>_n<N>.txt   report_timing_summary
#   summary.txt                one Markdown table with everything
#                              (bash noc_study/run_study.sh report copies it
#                              into noc_study/results.md)
#
# Fmax = 1000 / (period - WNS) MHz, WNS = worst setup slack of the clk paths.
# After synthesis only, the wire delays are estimates, so Fmax is a bit
# optimistic; use "impl" for routed numbers.
# =============================================================================

set root [file normalize [file join [file dirname [info script]] ..]]
set out  [file join $root noc_study vivado_reports]
file mkdir $out

set part    xc7z020clg484-1
set period  10.0
set do_impl 0
set sizes   {4 8 16}
set fabrics {bus xbar mesh}

if {[info exists argv]} {
    foreach a $argv {
        switch -glob -- $a {
            impl     { set do_impl 1 }
            quick    { set sizes {4 8} }
            period=* { set period [string range $a 7 end] }
            default  { puts "vivado_study: unknown option '$a' (ignored)" }
        }
    }
}

# source files (the mesh needs the router and the mesh of rtl/noc)
set src [list \
    [file join $root noc_study rtl study_bus.v] \
    [file join $root noc_study rtl study_xbar.v] \
    [file join $root noc_study rtl study_mesh.v] \
    [file join $root rtl noc noc_mesh.v] \
    [file join $root rtl noc noc_router.v]]

# clock constraint, written next to the reports
set xdc [file join $out study_clk.xdc]
set fh [open $xdc w]
puts $fh "create_clock -period $period -name clk \[get_ports clk\]"
close $fh

# first number in the row "| <label> | <used> |" of a utilization report
proc util_num {text label} {
    set re [format {\|\s*%s\s*\|\s*([0-9]+)} $label]
    if {[regexp $re $text -> v]} { return $v }
    return "?"
}

set rows   {}
set failed {}
set t_all  [clock seconds]

foreach n $sizes {
    foreach f $fabrics {
        set tag ${f}_n${n}
        puts ""
        puts "=================== $tag ==================="
        set t0 [clock seconds]
        if {[catch {
            # the sources stay loaded from one run to the next
            if {[llength [get_files -quiet study_bus.v]] == 0} {
                read_verilog $src
                read_xdc $xdc
            }
            synth_design -top study_$f -part $part -mode out_of_context \
                         -generic N=$n -flatten_hierarchy rebuilt
            if {$do_impl} {
                opt_design
                place_design
                route_design
            }
            report_utilization -file [file join $out util_$tag.txt]
            report_timing_summary -max_paths 5 -file [file join $out timing_$tag.txt]

            set util [report_utilization -return_string]
            set luts [util_num $util {Slice LUTs\*?}]
            set lmem [util_num $util {LUT as Memory}]
            set ffs  [util_num $util {Slice Registers}]

            set wns  "n/a"
            set fmax "n/a"
            set lvls "n/a"
            set paths [get_timing_paths -delay_type max -max_paths 1 -nworst 1]
            if {[llength $paths] > 0} {
                set p [lindex $paths 0]
                set s [get_property SLACK $p]
                if {$s ne ""} {
                    set wns  [format %.3f $s]
                    set fmax [format %.1f [expr {1000.0 / ($period - $s)}]]
                    set lvls [get_property LOGIC_LEVELS $p]
                }
            }
            lappend rows [list $n $f $luts $lmem $ffs $wns $fmax $lvls]
            puts "vivado_study: $tag  LUTs $luts (LUTRAM $lmem)  FFs $ffs  WNS $wns ns  Fmax $fmax MHz  levels $lvls  ([expr {[clock seconds] - $t0}] s)"
            close_design
        } err]} {
            puts "vivado_study: $tag FAILED: $err"
            lappend failed $tag
            lappend rows [list $n $f FAILED - - - - -]
            catch {close_design}
        }
    }
}

# ---------------------------------------------------------------------------
# summary table (Markdown, also easy to read as plain text)
# ---------------------------------------------------------------------------
set stage [expr {$do_impl ? "after place and route" : "after synthesis (estimated wire delays)"}]
set lines {}
lappend lines "Vivado [version -short], part $part, out of context, clock $period ns,"
lappend lines "numbers $stage, [clock format [clock seconds] -format %Y-%m-%d]."
lappend lines ""
lappend lines "| N | fabric | LUTs | of which LUTRAM | FFs | WNS (ns) | Fmax (MHz) | logic levels |"
lappend lines "|---:|---|---:|---:|---:|---:|---:|---:|"
foreach r $rows {
    lappend lines "| [join $r { | }] |"
}
set fh [open [file join $out summary.txt] w]
puts $fh [join $lines "\n"]
close $fh

puts ""
puts "=================== NoC study summary ==================="
puts [join $lines "\n"]
puts ""
puts "Reports in $out ([expr {([clock seconds] - $t_all) / 60}] minutes)"

if {[llength $failed] > 0} {
    error "vivado_study: these runs failed: $failed"
}
