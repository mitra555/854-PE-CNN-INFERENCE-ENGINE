# =============================================================================
# innovus_floorplan.tcl — Floorplan / Power / Pin Placement
# Design:  CNN_Inference_Engine (throughput-optimised)
# Tool:    Cadence Innovus
# Kit:     Cadence 45 nm (gsclib045), 11-metal stack
#
# Die: 604.6 x 604.58 µm | AR = 1.0 | util = 0.60 | 20 µm core margins
#
# Run order within Innovus:
#   1. source innovus_floorplan.tcl       ← this file
#   2. source innovus_place_route.tcl
#
# Project directory layout:
#   cnn_ie_pd/
#   ├── rtl/                              ← RTL sources
#   ├── lib/                              ← timing libraries
#   ├── lef/                              ← tech + macro LEF
#   ├── constraints/
#   │   ├── mmmc.tcl
#   │   └── cnn_ie_constraints.sdc
#   ├── synthesis/
#   │   └── reports/cnn_ie_netlist.v      ← netlist from Genus
#   └── pd/
#       ├── innovus_floorplan.tcl         ← this file
#       ├── innovus_place_route.tcl
#       └── reports/
# =============================================================================

set_multi_cpu_usage -localCpu 8


# -----------------------------------------------------------------------------
# 0. Design initialisation
#    Read synthesised netlist, LEF, and MMMC setup.
# -----------------------------------------------------------------------------
set init_verilog        ../synthesis/reports/cnn_ie_netlist.v
set init_design_set_top CNN_Inference_Engine
set init_mmmc_file      ../constraints/mmmc.tcl
set init_lef_file { \
    /home/ms2025019_vaishnavi/cadence/pdks/cadence_45nm/lef/gsclib045_tech.lef \
    /home/ms2025019_vaishnavi/cadence/pdks/cadence_45nm/lef/gsclib045_macro.lef \
}
set init_pwr_net VDD
set init_gnd_net VSS

setDesignMode -process 45
init_design
report_analysis_view


# -----------------------------------------------------------------------------
# 1. Full reset — wipe any prior placement, pins, power, floorplan state
# -----------------------------------------------------------------------------
setDesignMode -reset
setAttribute -net VDD -skip_routing true
setAttribute -net VSS -skip_routing true
dbSet [dbGet top.insts.pStatus] unplaced
deleteAllPowerPreroutes
editPin -fixedPin 0 -pin [dbGet top.terms.name]


# -----------------------------------------------------------------------------
# 2. Floorplan
#    IMPFP-3961 warnings about CornerSite/IOSite are harmless.
# -----------------------------------------------------------------------------
floorPlan -site CoreSite -r 1.0 0.60 20 20 20 20

checkFPlan
checkDesign -floorplan


# -----------------------------------------------------------------------------
# 3. Global net connect — must precede addRing / addStripe
# -----------------------------------------------------------------------------
globalNetConnect VDD -type pgpin -pin VDD -inst * -override -verbose
globalNetConnect VSS -type pgpin -pin VSS -inst * -override -verbose
globalNetConnect VDD -type tiehigh -override -verbose
globalNetConnect VSS -type tielow  -override -verbose


# -----------------------------------------------------------------------------
# 4. Power ring  (M10 left/right, M11 top/bottom)
#    spacing = 1.8 µm  (IMPPP-136 fix: min required = 1.25 µm)
#    Ring outer edge from die boundary = offset + width + spacing
#                                      = 1.0 + 1.5 + 1.8 = 4.3 µm
# -----------------------------------------------------------------------------
setAddRingMode \
    -stacked_via_top_layer    M11 \
    -stacked_via_bottom_layer M1

addRing \
    -nets    {VDD VSS} \
    -type    core_rings \
    -layer   {top M11 bottom M11 left M10 right M10} \
    -width   {top 1.5 bottom 1.5 left 1.5 right 1.5} \
    -spacing {top 1.8 bottom 1.8 left 1.8 right 1.8} \
    -offset  {top 1.0 bottom 1.0 left 1.0 right 1.0}


# -----------------------------------------------------------------------------
# 5. Power stripes  (vertical, M10)
#    -start_offset 30 µm clears the right-side ring edge (IMPPP-170 fix)
#    Ring right-edge ≈ 20 + 1.0 + 2*(1.5+1.8) = 27.6 µm → 30 µm is safe
# -----------------------------------------------------------------------------
addStripe \
    -nets                {VDD VSS} \
    -layer               M10 \
    -direction           vertical \
    -width               0.6 \
    -spacing             0.5 \
    -set_to_set_distance 30 \
    -start_from          left \
    -start_offset        30


# -----------------------------------------------------------------------------
# 6. Pin placement
#    Safe range [25 .. 580] clears ring-corner MetSpc DRC violations.
#    Layer convention (45 nm kit):
#      M3 horizontal-preferred → LEFT and RIGHT sides
#      M4 vertical-preferred   → TOP and BOTTOM sides
#    IMPDBTCL-246 "Deleting attribute vio_layer" warnings are cosmetic only.
#
#    Total: 34 (LEFT) + 32 (BOTTOM) + 54 (RIGHT) + 14 (TOP) = 134 pins
# -----------------------------------------------------------------------------
setPinAssignMode -pinEditInBatch true

# LEFT (34 pins): clk, rst, start, layer_config_valid/ack,
#                 FClast[7:0], N_classes[9:0], layer_comp_type[2:0], layer_num[7:0]
editPin \
    -side LEFT \
    -layer M3 \
    -fixedPin 1 \
    -spreadType RANGE \
    -start {0 25} \
    -end   {0 580} \
    -spreadDirection CounterClockwise \
    -pin { clk rst start layer_config_valid layer_config_ack \
           {FClast[7]} {FClast[6]} {FClast[5]} {FClast[4]} \
           {FClast[3]} {FClast[2]} {FClast[1]} {FClast[0]} \
           {N_classes[9]} {N_classes[8]} {N_classes[7]} {N_classes[6]} \
           {N_classes[5]} {N_classes[4]} {N_classes[3]} {N_classes[2]} \
           {N_classes[1]} {N_classes[0]} \
           {layer_comp_type[2]} {layer_comp_type[1]} {layer_comp_type[0]} \
           {layer_num[7]} {layer_num[6]} {layer_num[5]} {layer_num[4]} \
           {layer_num[3]} {layer_num[2]} {layer_num[1]} {layer_num[0]} }

# BOTTOM (32 pins): nl[15:0], rl[15:0]
editPin \
    -side BOTTOM \
    -layer M4 \
    -fixedPin 1 \
    -spreadType RANGE \
    -start {25 0} \
    -end   {580 0} \
    -spreadDirection CounterClockwise \
    -pin { {nl[15]} {nl[14]} {nl[13]} {nl[12]} {nl[11]} {nl[10]} \
           {nl[9]}  {nl[8]}  {nl[7]}  {nl[6]}  {nl[5]}  {nl[4]}  \
           {nl[3]}  {nl[2]}  {nl[1]}  {nl[0]}  \
           {rl[15]} {rl[14]} {rl[13]} {rl[12]} {rl[11]} {rl[10]} \
           {rl[9]}  {rl[8]}  {rl[7]}  {rl[6]}  {rl[5]}  {rl[4]}  \
           {rl[3]}  {rl[2]}  {rl[1]}  {rl[0]} }

# RIGHT (54 pins): dram_I[15:0]+valid+req, dram_W[15:0]+valid+req, dram_B[15:0]+valid+req
editPin \
    -side RIGHT \
    -layer M3 \
    -fixedPin 1 \
    -spreadType RANGE \
    -start {604.6 25} \
    -end   {604.6 580} \
    -spreadDirection Clockwise \
    -pin { {dram_I_data[15]} {dram_I_data[14]} {dram_I_data[13]} {dram_I_data[12]} \
           {dram_I_data[11]} {dram_I_data[10]} {dram_I_data[9]}  {dram_I_data[8]}  \
           {dram_I_data[7]}  {dram_I_data[6]}  {dram_I_data[5]}  {dram_I_data[4]}  \
           {dram_I_data[3]}  {dram_I_data[2]}  {dram_I_data[1]}  {dram_I_data[0]}  \
           dram_I_valid dram_I_req \
           {dram_W_data[15]} {dram_W_data[14]} {dram_W_data[13]} {dram_W_data[12]} \
           {dram_W_data[11]} {dram_W_data[10]} {dram_W_data[9]}  {dram_W_data[8]}  \
           {dram_W_data[7]}  {dram_W_data[6]}  {dram_W_data[5]}  {dram_W_data[4]}  \
           {dram_W_data[3]}  {dram_W_data[2]}  {dram_W_data[1]}  {dram_W_data[0]}  \
           dram_W_valid dram_W_req \
           {dram_B_data[15]} {dram_B_data[14]} {dram_B_data[13]} {dram_B_data[12]} \
           {dram_B_data[11]} {dram_B_data[10]} {dram_B_data[9]}  {dram_B_data[8]}  \
           {dram_B_data[7]}  {dram_B_data[6]}  {dram_B_data[5]}  {dram_B_data[4]}  \
           {dram_B_data[3]}  {dram_B_data[2]}  {dram_B_data[1]}  {dram_B_data[0]}  \
           dram_B_valid dram_B_req }

# TOP (14 pins): done_interrupt, CN_DC_out[9:0], state_out[2:0]
editPin \
    -side TOP \
    -layer M4 \
    -fixedPin 1 \
    -spreadType RANGE \
    -start {25 604.58} \
    -end   {400 604.58} \
    -spreadDirection Clockwise \
    -pin { done_interrupt \
           {CN_DC_out[9]} {CN_DC_out[8]} {CN_DC_out[7]} {CN_DC_out[6]} \
           {CN_DC_out[5]} {CN_DC_out[4]} {CN_DC_out[3]} {CN_DC_out[2]} \
           {CN_DC_out[1]} {CN_DC_out[0]} \
           {state_out[2]} {state_out[1]} {state_out[0]} }

setPinAssignMode -pinEditInBatch false


# -----------------------------------------------------------------------------
# 7. Verify pin assignment — target: 134 legal, 0 illegal
# -----------------------------------------------------------------------------
checkPinAssignment

set illegal_pins [dbGet [dbGet -p top.terms.isIllegal 1].name]
if {[llength $illegal_pins] > 0} {
    puts "*** ILLEGAL PINS: $illegal_pins"
} else {
    puts "*** All 134 pins legal — checkPinAssignment PASSED ***"
}


# -----------------------------------------------------------------------------
# 8. DRC check on power grid  (target: 0 violations)
# -----------------------------------------------------------------------------
verify_drc
verify_drc -check_short_only


# =============================================================================
# End of floorplan / power / pin script.
#
# verifyConnectivity is intentionally omitted here.
# IMPVFC-96 unplaced-instance warnings before place_design are all false
# positives and produce no actionable information at this stage.
#
# Continue with:  source innovus_place_route.tcl
# =============================================================================
