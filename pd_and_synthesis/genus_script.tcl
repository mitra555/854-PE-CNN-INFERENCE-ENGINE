# -----------------------------------------------------------------------------
# 1. Library and RTL search paths
# -----------------------------------------------------------------------------
set_attr init_lib_search_path ../lib/
set_attr hdl_search_path      ../rtl/

# Synthesis uses the slow (worst-case) library to be conservative on timing
set_attr library slow_vdd1v0_basicCells.lib


# -----------------------------------------------------------------------------
# 2. Read RTL  (filenames match the rtl/ folder)
# -----------------------------------------------------------------------------
read_hdl { \
    kpu.sv \
    cu.sv \
    iec.sv \
    cnn_inference_engine.sv \
}


# -----------------------------------------------------------------------------
# 3. Elaborate and set top module
# -----------------------------------------------------------------------------
elaborate
set_top_module CNN_Inference_Engine


# -----------------------------------------------------------------------------
# 4. Timing constraints
#    Target: 3.85 GHz (ASIC) → period ≈ 0.26 ns
# -----------------------------------------------------------------------------
read_sdc ../constraints/cnn_ie_constraints.sdc


# -----------------------------------------------------------------------------
# 5. Generic synthesis  (technology-independent)
# -----------------------------------------------------------------------------
set_attr syn_generic_effort medium
syn_generic


# -----------------------------------------------------------------------------
# 6. Technology mapping
# -----------------------------------------------------------------------------
syn_map


# -----------------------------------------------------------------------------
# 7. Gate-level optimisation
# -----------------------------------------------------------------------------
set_attr syn_opt_effort medium
syn_opt


# -----------------------------------------------------------------------------
# 8. Reports  (written to synthesis/reports/)
# -----------------------------------------------------------------------------
check_design  > ./reports/design_check.txt
report_area   > ./reports/area.txt
report_power  > ./reports/power.txt
report_timing > ./reports/timing.txt
report_gates  > ./reports/gates.txt
report_hierarchy


# -----------------------------------------------------------------------------
# 9. Netlist and SDC outputs
# -----------------------------------------------------------------------------
write_hdl > ./reports/cnn_ie_netlist.v
write_sdc > ./reports/cnn_ie_out.sdc


# -----------------------------------------------------------------------------
# Type 'quit' to release licences — never use Ctrl+C.
# -----------------------------------------------------------------------------
