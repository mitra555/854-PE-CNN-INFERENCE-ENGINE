`timescale 1ns/1ps
// =============================================================
// IEC — Throughput-Optimised Inference Engine Controller
// Paper: Islam et al., TCAS-I 2024, §II-A, Algorithm 1
//
// Throughput fixes vs energy-optimised baseline (THR-IEC-x):
//   THR-IEC-1  Overlapped PREFETCH + COMPUTE (Alg-1 L5-15).
//              KPC handles PREFETCH→COMPUTE autonomously once
//              r_min items are received; IEC does not stall PEs.
//   THR-IEC-2  Tail-overlap: KPC transitions to ST_STR_NEXT
//              autonomously (Alg-1 L16-22); IEC only monitors
//              layer_done.
//   THR-IEC-3  psum_reg accumulates across all nl iterations;
//              reset only on layer start (Alg-1 L12).
//   THR-IEC-4  K-wide serial weight bus replaces M×N×K broadcast.
//   THR-IEC-5  layer_start pulsed to CU on every S_CONFIG entry
//              (THR-CU-5 clear of class_done).
//   THR-IEC-6  No warmup counter needed; conv_valid from KPU is
//              already aligned to the pipelined adder_tree.
//
// FSM (Gray-coded):
//   S_IDLE         000
//   S_CONFIG       001
//   S_PREFETCH     011
//   S_COMPUTE      010
//   S_LAYER_SWITCH 110
//   S_CLASSIFY     111
//   S_DONE         101
// =============================================================


package pkg_IEC;
    parameter int K          = 16;
    parameter int M          = 9;
    parameter int N_PE       = 96;
    parameter int N_WIDTH    = 10;
    parameter int L_WIDTH    = 8;
    parameter int NL_WIDTH   = 16;
    parameter int COMP_WIDTH = 3;

    // Computation type codes (must match pkg_KPU::comp_t)
    parameter logic [COMP_WIDTH-1:0] COMP_CONV    = 3'd0;
    parameter logic [COMP_WIDTH-1:0] COMP_FC      = 3'd1;
    parameter logic [COMP_WIDTH-1:0] COMP_MAXPOOL = 3'd2;
    parameter logic [COMP_WIDTH-1:0] COMP_AVGPOOL = 3'd3;
    parameter logic [COMP_WIDTH-1:0] COMP_RELU    = 3'd4;
    parameter logic [COMP_WIDTH-1:0] COMP_RELU6   = 3'd5;
endpackage


module IEC
import pkg_IEC::*;
(
    input  logic                    clk,
    input  logic                    rst,
    input  logic                    start,

    // Layer configuration from on-chip processor
    input  logic                    layer_config_valid,
    input  logic [L_WIDTH-1:0]      FClast,
    input  logic [N_WIDTH-1:0]      N_classes,
    input  logic [COMP_WIDTH-1:0]   layer_comp_type,
    input  logic [NL_WIDTH-1:0]     nl,
    input  logic [NL_WIDTH-1:0]     rl,
    input  logic [L_WIDTH-1:0]      layer_num,

    // Status to processor
    output logic                    done_interrupt,
    output logic                    layer_config_ack,
    output logic [N_WIDTH-1:0]      CN_DC_proc,
    output logic [2:0]              state_out,

    // KPU configuration outputs
    output logic [COMP_WIDTH-1:0]   kpu_comp_type,
    output logic                    kpu_layer_start,
    output logic                    kpu_S_Ovd,

    // Data bus to KPU (K-wide serial, THR-IEC-4)
    output logic signed [K-1:0]     kpu_I,
    output logic                    kpu_I_valid,
    output logic signed [K-1:0]     kpu_W,
    output logic                    kpu_W_valid,
    output logic signed [K-1:0]     kpu_B [0:M-1],
    output logic                    kpu_B_valid,

    // KPU status inputs
    input  logic signed [K+$clog2(M+1)-1:0] kpu_conv_out,
    input  logic                    kpu_conv_valid,
    input  logic                    kpu_layer_done,

    // CU interface
    output logic signed [K-1:0]     cu_AC_in,
    output logic                    cu_valid,
    output logic                    cu_l_is_FClast,
    output logic [N_WIDTH-1:0]      cu_N,
    output logic                    cu_layer_start,
    input  logic [N_WIDTH-1:0]      cu_CN_DC,
    input  logic                    cu_result_valid,
    input  logic signed [K-1:0]     cu_AC_passthrough,
    input  logic                    cu_valid_passthrough,

    // DRAM interface
    input  logic signed [K-1:0]     dram_I_data,
    input  logic                    dram_I_valid,
    output logic                    dram_I_req,

    input  logic signed [K-1:0]     dram_W_data,
    input  logic                    dram_W_valid,
    output logic                    dram_W_req,

    input  logic signed [K-1:0]     dram_B_data,
    input  logic                    dram_B_valid,
    output logic                    dram_B_req
);
    // FSM state encoding (Gray-coded)
    localparam logic [2:0]
        S_IDLE         = 3'b000,
        S_CONFIG       = 3'b001,
        S_PREFETCH     = 3'b011,
        S_COMPUTE      = 3'b010,
        S_LAYER_SWITCH = 3'b110,
        S_CLASSIFY     = 3'b111,
        S_DONE         = 3'b101;

    logic [2:0] state;
    assign state_out = state;

    // Registered layer parameters
    logic [L_WIDTH-1:0]    r_FClast;
    logic [N_WIDTH-1:0]    r_N_classes;
    logic [COMP_WIDTH-1:0] r_comp_type;
    logic [NL_WIDTH-1:0]   r_nl;
    logic [NL_WIDTH-1:0]   r_rl;
    logic [L_WIDTH-1:0]    r_layer;

    // Counters
    logic [NL_WIDTH-1:0]   iter_cnt;
    logic [NL_WIDTH-1:0]   classify_cnt;

    // THR-IEC-3: psum accumulator across iterations
    localparam int PSUM_W = K + $clog2(M + 1);
    logic signed [PSUM_W-1:0] psum_reg;

    wire l_is_FClast = (r_layer == r_FClast);
    wire last_iter   = (iter_cnt == r_nl - 1'b1);

    // Main FSM
    always_ff @(posedge clk) begin
        if (rst) begin
            state            <= S_IDLE;
            iter_cnt         <= '0;
            classify_cnt     <= '0;
            psum_reg         <= '0;
            done_interrupt   <= 1'b0;
            layer_config_ack <= 1'b0;
            CN_DC_proc       <= '0;
            kpu_layer_start  <= 1'b0;
            cu_layer_start   <= 1'b0;
        end else begin
            layer_config_ack <= 1'b0;
            kpu_layer_start  <= 1'b0;
            cu_layer_start   <= 1'b0;

            case (state)

                S_IDLE: begin
                    done_interrupt <= 1'b0;
                    classify_cnt   <= '0;
                    if (start) state <= S_CONFIG;
                end

                S_CONFIG: begin
                    if (layer_config_valid) begin
                        r_FClast         <= FClast;
                        r_N_classes      <= N_classes;
                        r_comp_type      <= layer_comp_type;
                        r_nl             <= nl;
                        r_rl             <= rl;
                        r_layer          <= layer_num;
                        iter_cnt         <= '0;
                        classify_cnt     <= '0;
                        psum_reg         <= '0;
                        layer_config_ack <= 1'b1;
                        kpu_layer_start  <= 1'b1;   // THR-IEC-5
                        cu_layer_start   <= 1'b1;   // THR-CU-5
                        state            <= S_PREFETCH;
                    end
                end

                // IEC waits; KPC drives pre-fetch autonomously (THR-IEC-1)
                S_PREFETCH: begin
                    if (kpu_conv_valid) state <= S_COMPUTE;
                end

                S_COMPUTE: begin
                    if (kpu_conv_valid) begin
                        psum_reg <= psum_reg + PSUM_W'(signed'(kpu_conv_out));
                        if (last_iter) begin
                            if (l_is_FClast) state <= S_CLASSIFY;
                            else             state <= S_LAYER_SWITCH;
                        end else
                            iter_cnt <= iter_cnt + 1'b1;
                    end
                    if (kpu_layer_done && !kpu_conv_valid) begin
                        if (l_is_FClast) state <= S_CLASSIFY;
                        else             state <= S_LAYER_SWITCH;
                    end
                end

                S_LAYER_SWITCH: begin
                    iter_cnt <= '0;
                    psum_reg <= '0;
                    state    <= S_CONFIG;
                end

                S_CLASSIFY: begin
                    if (classify_cnt < r_N_classes)
                        classify_cnt <= classify_cnt + 1'b1;
                    if (cu_result_valid) begin
                        CN_DC_proc <= cu_CN_DC;
                        state      <= S_DONE;
                    end
                end

                S_DONE: begin
                    done_interrupt <= 1'b1;
                    classify_cnt   <= '0;
                    if (!start) state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // DRAM request signals
    assign dram_I_req = (state == S_PREFETCH || state == S_COMPUTE);
    assign dram_W_req = (state == S_CONFIG);
    assign dram_B_req = (state == S_CONFIG);

    // KPU data buses (THR-IEC-4: K-wide serial)
    wire i_active = (state == S_PREFETCH || state == S_COMPUTE);
    wire w_active = (state == S_CONFIG);

    assign kpu_I       = i_active ? dram_I_data : '0;
    assign kpu_I_valid = i_active ? dram_I_valid : 1'b0;
    assign kpu_W       = w_active ? dram_W_data : '0;
    assign kpu_W_valid = w_active ? dram_W_valid : 1'b0;
    assign kpu_B_valid = w_active ? dram_B_valid : 1'b0;

    genvar gi;
    generate
        for (gi = 0; gi < M; gi++) begin : BIAS_HOLD
            assign kpu_B[gi] = w_active ? dram_B_data : '0;
        end
    endgenerate

    // KPU configuration
    assign kpu_comp_type = r_comp_type;
    assign kpu_S_Ovd     = (r_layer == '0);

    // CU interface
    assign cu_AC_in       = (state == S_CLASSIFY) ? psum_reg[K-1:0] : '0;
    assign cu_valid       = (state == S_CLASSIFY) && (classify_cnt < r_N_classes);
    assign cu_l_is_FClast = (state == S_CLASSIFY);
    assign cu_N           = r_N_classes;

endmodule
