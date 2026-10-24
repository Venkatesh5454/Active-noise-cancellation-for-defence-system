# =============================================================================
# sources.tcl - the list of design files, shared by every v2 build script
# (sourced by the other scripts; sets $rtl_files and $sim_model_files)
# =============================================================================
set rtl_files {}
foreach d {v1 noc se xbar hub top} {
    foreach f [lsort [glob -nocomplain [file join $root rtl $d *.v]]] { lappend rtl_files $f }
}
set sim_model_files [lsort [glob -nocomplain [file join $root sim models *.v]]]

# Implementation settings shared by the build scripts (timing margin at
# 100 MHz): physical optimisation after placement (directive Explore) and
# again after routing.  Both only touch paths that are short of time
# (replicate high-fanout drivers, re-place critical cells ...) and add a
# few minutes to the run.
proc ssc2_impl_settings {run} {
    foreach {prop val} {
        STEPS.PHYS_OPT_DESIGN.IS_ENABLED            true
        STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE        Explore
        STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true
    } {
        if {[catch {set_property $prop $val [get_runs $run]} msg]} {
            puts "NOTE: could not set $prop on $run ($msg)"
        }
    }
}
