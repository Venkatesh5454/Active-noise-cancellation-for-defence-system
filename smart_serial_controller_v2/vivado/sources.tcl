# =============================================================================
# sources.tcl - the list of design files, shared by every v2 build script
# (sourced by the other scripts; sets $rtl_files and $sim_model_files)
# =============================================================================
set rtl_files {}
foreach d {v1 noc se xbar hub top} {
    foreach f [lsort [glob -nocomplain [file join $root rtl $d *.v]]] { lappend rtl_files $f }
}
set sim_model_files [lsort [glob -nocomplain [file join $root sim models *.v]]]
