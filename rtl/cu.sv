`timescale 1ns / 1ps
// =============================================================
// CU — Throughput-Optimised Classify Unit
// Paper: Islam et al., TCAS-I 2024, §III-B, Algorithm 1
//
// Modules in this file
//   pkg_CU  — parameter package
//   CNG     — Class Number Generator            (§III-B.2)
//   ACSU    — Activation Searching Unit         (§III-B.3)
//   CUC     — Classify Unit Controller          (§III-B.2)
//   DSR     — Data & Signal Router              (§III-B.1)
//   CU      — Top-level wrapper                 (Fig. 7)
//
// Throughput fixes vs energy-optimised baseline (THR-CU-x):
//   THR-CU-1  ACSU update condition is >= (not >) so class 0
//             is always captured (Alg-1 lines 25-26).
//   THR-CU-2  ACMax initialised to most-negative K-bit signed
//             value, safe when FClast activations are negative.
//   THR-CU-3  CNG is registered and increments on the same
//             enable pulse as ACSU, eliminating a 1-cycle CN/AC
//             mismatch (class 0 was tagged with CN=1 in baseline).
//   THR-CU-4  DSR outputs registered, removing DSR from the
//             combinational critical path to IEC.
//   THR-CU-5  CUC class_done clears on layer_start (not only
//             rst) for back-to-back inference without global reset.
// =============================================================


package pkg_CU;
    parameter int K       = 16;   // activation bit-width
    parameter int N_WIDTH = 10;   // log₂(max classes): 10 → up to 1024
endpackage


// =============================================================
// CNG — Class Number Generator  (§III-B.2)
//
// Synchronous up-counter; registered (THR-CU-3) so CN is valid
// on the same clock edge that triggered ACSU.
// =============================================================
module CNG
import pkg_CU::*;
(
    input  logic               clk,
    input  logic               rst,
    input  logic               en,
    output logic [N_WIDTH-1:0] CN
);
    always_ff @(posedge clk) begin
        if (rst)    CN <= '0;
        else if (en) CN <= CN + 1'b1;
    end
endmodule


// =============================================================
// ACSU — Activation Searching Unit  (§III-B.3, Fig. 7, Alg-1 L25-26)
//
// THR-CU-1: update when ACi >= ACMax  (subtractor-based >= test).
// THR-CU-2: ACMax initialised to most-negative K-bit signed value.
// =============================================================
module ACSU
import pkg_CU::*;
(
    input  logic                   clk,
    input  logic                   rst,
    input  logic                   en,
    input  logic                   AC_valid,
    input  logic signed [K-1:0]   ACi,
    input  logic [N_WIDTH-1:0]    CNi,
    output logic [N_WIDTH-1:0]    CN_DC
);
    localparam logic signed [K-1:0] AC_MIN = {1'b1, {(K-1){1'b0}}};

    logic signed [K-1:0] ACMax;
    logic [N_WIDTH-1:0]  CN_DC_r;

    // diff[K-1] == 0 means ACi - ACMax >= 0, i.e. ACi >= ACMax
    wire signed [K-1:0] diff   = ACi - ACMax;
    wire                update = en && AC_valid && !diff[K-1];

    always_ff @(posedge clk) begin
        if (rst) begin
            ACMax   <= AC_MIN;
            CN_DC_r <= '0;
        end else if (update) begin
            ACMax   <= ACi;
            CN_DC_r <= CNi;
        end
    end

    assign CN_DC = CN_DC_r;
endmodule


// =============================================================
// CUC — Classify Unit Controller  (§III-B.2, Fig. 7)
//
// THR-CU-5: class_done clears on layer_start for back-to-back
// inference without a global reset.
// =============================================================
module CUC
import pkg_CU::*;
(
    input  logic               clk,
    input  logic               rst,
    input  logic               layer_start,
    input  logic               l_is_FClast,
    input  logic               AC_valid,
    input  logic [N_WIDTH-1:0] CN_count,
    input  logic [N_WIDTH-1:0] N,
    output logic               acsu_cng_en,
    output logic               class_done
);
    assign acsu_cng_en = l_is_FClast && AC_valid && (CN_count < N) && (N != '0);

    wire cn_reached = l_is_FClast && (N != '0) && (CN_count == N);

    always_ff @(posedge clk) begin
        if (rst || layer_start) class_done <= 1'b0;
        else if (cn_reached)    class_done <= 1'b1;
    end
endmodule


// =============================================================
// DSR — Data & Signal Router  (§III-B.1, Fig. 7)
//
// THR-CU-4: all outputs registered (one FF stage) to remove
// DSR from the combinational critical path to IEC.
// =============================================================
module DSR
import pkg_CU::*;
(
    input  logic                  clk,
    input  logic                  rst,

    input  logic signed [K-1:0]  AC_Psum_in,
    input  logic                  valid_in,
    input  logic                  l_is_FClast,

    input  logic [N_WIDTH-1:0]   CN_DC,
    input  logic                  class_done,

    output logic signed [K-1:0]  AC_to_ACSU,
    output logic                  valid_to_CUC,

    output logic signed [K-1:0]  AC_out,
    output logic [N_WIDTH-1:0]   CN_DC_out,
    output logic                  valid_out,
    output logic                  result_valid
);
    always_ff @(posedge clk) begin
        if (rst) begin
            AC_to_ACSU   <= '0;
            valid_to_CUC <= 1'b0;
            AC_out       <= '0;
            CN_DC_out    <= '0;
            valid_out    <= 1'b0;
            result_valid <= 1'b0;
        end else begin
            AC_to_ACSU   <= l_is_FClast ? AC_Psum_in : '0;
            valid_to_CUC <= l_is_FClast ? valid_in   : 1'b0;
            AC_out       <= l_is_FClast ? '0         : AC_Psum_in;
            valid_out    <= l_is_FClast ? 1'b0       : valid_in;
            CN_DC_out    <= CN_DC;
            result_valid <= class_done;
        end
    end
endmodule


// =============================================================
// CU — top-level wrapper  (§III-B, Fig. 7)
// =============================================================
module CU
import pkg_CU::*;
(
    input  logic                  clk,
    input  logic                  rst,
    input  logic                  layer_start,

    input  logic signed [K-1:0]  AC_Psum_in,
    input  logic                  valid_in,
    input  logic                  l_is_FClast,
    input  logic [N_WIDTH-1:0]   N,

    output logic signed [K-1:0]  AC_out,
    output logic                  valid_out,
    output logic [N_WIDTH-1:0]   CN_DC_out,
    output logic                  result_valid
);
    wire signed [K-1:0]  AC_to_ACSU;
    wire                  valid_to_CUC;
    wire                  acsu_cng_en;
    wire                  class_done;
    wire [N_WIDTH-1:0]    CN_from_CNG;
    wire [N_WIDTH-1:0]    CN_DC_from_ACSU;

    DSR dsr_inst (
        .clk          (clk),
        .rst          (rst),
        .AC_Psum_in   (AC_Psum_in),
        .valid_in     (valid_in),
        .l_is_FClast  (l_is_FClast),
        .CN_DC        (CN_DC_from_ACSU),
        .class_done   (class_done),
        .AC_to_ACSU   (AC_to_ACSU),
        .valid_to_CUC (valid_to_CUC),
        .AC_out       (AC_out),
        .CN_DC_out    (CN_DC_out),
        .valid_out    (valid_out),
        .result_valid (result_valid)
    );

    CUC cuc_inst (
        .clk          (clk),
        .rst          (rst),
        .layer_start  (layer_start),
        .l_is_FClast  (l_is_FClast),
        .AC_valid     (valid_to_CUC),
        .CN_count     (CN_from_CNG),
        .N            (N),
        .acsu_cng_en  (acsu_cng_en),
        .class_done   (class_done)
    );

    CNG cng_inst (
        .clk (clk),
        .rst (rst),
        .en  (acsu_cng_en),
        .CN  (CN_from_CNG)
    );

    ACSU acsu_inst (
        .clk      (clk),
        .rst      (rst),
        .en       (acsu_cng_en),
        .AC_valid (valid_to_CUC),
        .ACi      (AC_to_ACSU),
        .CNi      (CN_from_CNG),
        .CN_DC    (CN_DC_from_ACSU)
    );

endmodule
