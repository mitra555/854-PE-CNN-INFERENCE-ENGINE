`timescale 1ns / 1ps
// =============================================================
// CNN_Inference_Engine — Throughput-Optimised Top-Level
// Paper: Islam et al., TCAS-I 2024, §II-A (Fig. 1), §IV
//
// Module hierarchy:
//   CNN_Inference_Engine
//     ├── IEC           (iec_throughput.sv)
//     ├── KPU_cluster   (kpu_throughput.sv)
//     └── CU            (cu_throughput.sv)
//
// Interface summary:
//   Processor side  — layer config handshake + done interrupt
//   DRAM side       — separate I / W / B handshake channels
//   Result          — CN_DC_out, state_out
// =============================================================

module CNN_Inference_Engine
(
    input  logic                              clk,
    input  logic                              rst,

    // Processor interface
    input  logic                              start,
    input  logic                              layer_config_valid,
    output logic                              layer_config_ack,
    input  logic [pkg_IEC::L_WIDTH-1:0]      FClast,
    input  logic [pkg_IEC::N_WIDTH-1:0]      N_classes,
    input  logic [pkg_IEC::COMP_WIDTH-1:0]   layer_comp_type,
    input  logic [pkg_IEC::NL_WIDTH-1:0]     nl,
    input  logic [pkg_IEC::NL_WIDTH-1:0]     rl,
    input  logic [pkg_IEC::L_WIDTH-1:0]      layer_num,
    output logic                              done_interrupt,
    output logic [pkg_IEC::N_WIDTH-1:0]      CN_DC_out,
    output logic [2:0]                        state_out,

    // DRAM interface (simple valid/request handshake)
    input  logic signed [pkg_KPU::K-1:0]    dram_I_data,
    input  logic                              dram_I_valid,
    output logic                              dram_I_req,

    input  logic signed [pkg_KPU::K-1:0]    dram_W_data,
    input  logic                              dram_W_valid,
    output logic                              dram_W_req,

    input  logic signed [pkg_KPU::K-1:0]    dram_B_data,
    input  logic                              dram_B_valid,
    output logic                              dram_B_req
);

    // ---- IEC ↔ KPU wires ----------------------------------------
    wire [pkg_IEC::COMP_WIDTH-1:0]  kpu_comp_type;
    wire                              kpu_layer_start;
    wire                              kpu_S_Ovd;

    wire signed [pkg_KPU::K-1:0]   kpu_I;
    wire                              kpu_I_valid;
    wire signed [pkg_KPU::K-1:0]   kpu_W;
    wire                              kpu_W_valid;
    wire signed [pkg_KPU::K-1:0]   kpu_B [0:pkg_KPU::M-1];
    wire                              kpu_B_valid;

    localparam int PSUM_W = pkg_KPU::K + $clog2(pkg_KPU::M + 1);
    wire signed [PSUM_W-1:0]         kpu_conv_out;
    wire                              kpu_conv_valid;
    wire                              kpu_layer_done;

    // ---- IEC ↔ CU wires -----------------------------------------
    wire signed [pkg_KPU::K-1:0]   cu_AC_in;
    wire                              cu_valid;
    wire                              cu_l_is_FClast;
    wire [pkg_IEC::N_WIDTH-1:0]     cu_N;
    wire                              cu_layer_start;
    wire [pkg_IEC::N_WIDTH-1:0]     cu_CN_DC;
    wire                              cu_result_valid;
    wire signed [pkg_KPU::K-1:0]   cu_AC_passthrough;
    wire                              cu_valid_passthrough;

    // ---- IEC -------------------------------------------------------
    IEC iec_inst (
        .clk                  (clk),
        .rst                  (rst),
        .start                (start),
        .layer_config_valid   (layer_config_valid),
        .FClast               (FClast),
        .N_classes            (N_classes),
        .layer_comp_type      (layer_comp_type),
        .nl                   (nl),
        .rl                   (rl),
        .layer_num            (layer_num),
        .done_interrupt       (done_interrupt),
        .layer_config_ack     (layer_config_ack),
        .CN_DC_proc           (CN_DC_out),
        .state_out            (state_out),
        .kpu_comp_type        (kpu_comp_type),
        .kpu_layer_start      (kpu_layer_start),
        .kpu_S_Ovd            (kpu_S_Ovd),
        .kpu_I                (kpu_I),
        .kpu_I_valid          (kpu_I_valid),
        .kpu_W                (kpu_W),
        .kpu_W_valid          (kpu_W_valid),
        .kpu_B                (kpu_B),
        .kpu_B_valid          (kpu_B_valid),
        .kpu_conv_out         (kpu_conv_out),
        .kpu_conv_valid       (kpu_conv_valid),
        .kpu_layer_done       (kpu_layer_done),
        .cu_AC_in             (cu_AC_in),
        .cu_valid             (cu_valid),
        .cu_l_is_FClast       (cu_l_is_FClast),
        .cu_N                 (cu_N),
        .cu_layer_start       (cu_layer_start),
        .cu_CN_DC             (cu_CN_DC),
        .cu_result_valid      (cu_result_valid),
        .cu_AC_passthrough    (cu_AC_passthrough),
        .cu_valid_passthrough (cu_valid_passthrough),
        .dram_I_data          (dram_I_data),
        .dram_I_valid         (dram_I_valid),
        .dram_I_req           (dram_I_req),
        .dram_W_data          (dram_W_data),
        .dram_W_valid         (dram_W_valid),
        .dram_W_req           (dram_W_req),
        .dram_B_data          (dram_B_data),
        .dram_B_valid         (dram_B_valid),
        .dram_B_req           (dram_B_req)
    );

    // ---- KPU_cluster -----------------------------------------------
    KPU_cluster #(
        .M_ROWS (pkg_KPU::M),
        .N_COLS (pkg_KPU::N),
        .BIT_W  (pkg_KPU::K),
        .Z_FILT (pkg_KPU::Z),
        .A_SIZE (pkg_KPU::A),
        .S_STEP (pkg_KPU::S),
        .R_MIN  (pkg_KPU::R_VAL),
        .AVG_SHF(pkg_KPU::AVG_SHIFT)
    ) kpu_inst (
        .clk          (clk),
        .rst          (rst),
        .layer_start  (kpu_layer_start),
        .comp_type    (pkg_KPU::comp_t'(kpu_comp_type)),
        .delta        (pkg_KPU::Z_BW'(pkg_KPU::Z)),
        .r_min        (pkg_KPU::R_BW'(pkg_KPU::R_VAL)),
        .A_size       (pkg_KPU::A_BW'(pkg_KPU::A)),
        .alpha        (pkg_KPU::M_BW'(pkg_KPU::M)),
        .I_data       (kpu_I),
        .I_valid      (kpu_I_valid),
        .W_data       (kpu_W),
        .W_valid      (kpu_W_valid),
        .B_data       (kpu_B),
        .B_valid      (kpu_B_valid),
        .S_Ovd        (kpu_S_Ovd),
        .clip6        (16'sd6),
        .conv_out     (kpu_conv_out),
        .conv_valid   (kpu_conv_valid),
        .layer_done   (kpu_layer_done)
    );

    // ---- CU --------------------------------------------------------
    CU cu_inst (
        .clk          (clk),
        .rst          (rst),
        .layer_start  (cu_layer_start),
        .AC_Psum_in   (cu_AC_in),
        .valid_in     (cu_valid),
        .l_is_FClast  (cu_l_is_FClast),
        .N            (cu_N),
        .AC_out       (cu_AC_passthrough),
        .valid_out    (cu_valid_passthrough),
        .CN_DC_out    (cu_CN_DC),
        .result_valid (cu_result_valid)
    );

endmodule
