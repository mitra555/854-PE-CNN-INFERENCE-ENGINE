# =============================================================================
# innovus_place_route.tcl — Placement / CTS / Routing / Sign-off
# Design:  CNN_Inference_Engine (throughput-optimised)
# Tool:    Cadence Innovus
# Kit:     Cadence 45 nm (gsclib045), 11-metal stack
#
# Prerequisite: innovus_floorplan.tcl must have completed cleanly
#   (0 DRC violations, 134 legal pins) before sourcing this script.
#
# MMMC views used (defined in mmmc.tcl):
#   VIEW_SETUP  → slow cells + worst-case RC  → setup sign-off
#   VIEW_HOLD   → fast cells + best-case  RC  → hold  sign-off
# =============================================================================

set_multi_cpu_usage -localCpu 8


# =============================================================================
# STEP 1 — SROUTE (core pins only — valid now that floorplan is complete)
# =============================================================================
sroute \
    -connect          {corePin floatingStripe} \
    -allowJogging     true \
    -allowLayerChange true \
    -targetViaLayerRange {M1 M11}


# =============================================================================
# STEP 2 — PLACEMENT
# =============================================================================
setPlaceMode \
    -place_global_cong_effort   high \
    -place_global_timing_effort high

place_design

# Pre-CTS timing baseline — setup on slow corner, hold on fast corner
setAnalysisMode -analysisType onChipVariation -cppr both
timeDesign -preCTS        -reportOnly
timeDesign -preCTS -hold  -reportOnly

# Pre-CTS optimisation
optDesign -preCTS

verify_drc
verifyConnectivity -type all
checkPinAssignment

saveDesign ./checkpoints/post_place.enc


# =============================================================================
# STEP 3 — CLOCK TREE SYNTHESIS
# =============================================================================
create_ccopt_clock_tree_spec
clock_opt_design -outDir ./reports/clock_reports

timeDesign -postCTS       -reportOnly
timeDesign -postCTS -hold -reportOnly

# Post-CTS setup optimisation
optDesign -postCTS

# Post-CTS hold fixing
setOptMode -opt_hold_allow_setup_tns_degradation true
optDesign -postCTS -hold
optDesign -postCTS

timeDesign -postCTS
timeDesign -postCTS -hold

saveDesign ./checkpoints/post_cts.enc


# =============================================================================
# STEP 4 — ROUTING
# =============================================================================
setNanoRouteMode -route_antenna_diode_insertion true
setNanoRouteMode -route_antenna_cell_name       ANTENNA
setNanoRouteMode -route_detail_fix_antenna      true

routeDesign -globalDetail

timeDesign -postRoute
timeDesign -postRoute -hold


# =============================================================================
# STEP 5 — POST-ROUTE OPTIMISATION
# =============================================================================
setOptMode -opt_hold_allow_setup_tns_degradation true
optDesign -postRoute
optDesign -postRoute -hold

# Incremental timing-driven ECO
setUsefulSkewMode -opt_skew_eco_route true
setOptMode -opt_setup_target_slack 0.4
setOptMode -effort high
setOptMode -fixFanoutLoad true
setOptMode -opt_hold_allow_setup_tns_degradation true
optDesign -postRoute -setup -hold -incr

# Final setup push
setOptMode -opt_hold_allow_setup_tns_degradation false
setOptMode -opt_setup_target_slack 0.05
optDesign -postRoute -setup -incr

setOptMode -opt_setup_target_slack 0.0
timeDesign -postRoute
timeDesign -postRoute -hold

saveDesign ./checkpoints/pre_filler.enc


# =============================================================================
# STEP 6 — FILLER INSERTION AND SIGN-OFF CHECKS
# =============================================================================
addFiller \
    -cell   {FILL64 FILL32 FILL16 FILL8 FILL4 FILL2 FILL1} \
    -prefix FILLER

verifyConnectivity -type all
verify_drc
verifyProcessAntenna

# DRV fix loop (re-runs only if violations remain after filler)
set drv_violations [verify_drc -reportAllViols -quiet]
if {$drv_violations > 0} {
    deleteFiller
    optDesign -postRoute -drv
    addFiller \
        -cell   {FILL64 FILL32 FILL16 FILL8 FILL4 FILL2 FILL1} \
        -prefix FILLER
    verifyConnectivity -type all
    verify_drc
    timeDesign -postRoute
    timeDesign -postRoute -hold
}


# =============================================================================
# STEP 7 — FINAL REPORTS  (setup uses slow corner, hold uses fast corner)
# =============================================================================
setAnalysisMode -analysisType onChipVariation -cppr both

# Setup — VIEW_SETUP (slow cells + worst-case RC)
report_timing -view VIEW_SETUP > ./reports/timing_postroute_setup.txt

# Hold — VIEW_HOLD (fast cells + best-case RC)
setAnalysisMode -checkType hold
report_timing -view VIEW_HOLD  > ./reports/timing_postroute_hold.txt

reportCongestion                > ./reports/congestion.txt
reportPowerPlan                 > ./reports/power_plan.txt

saveDesign ./checkpoints/post_route_final.enc

# =============================================================================
# End of place / CTS / route script.
# =============================================================================
