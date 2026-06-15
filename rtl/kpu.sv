`timescale 1ns / 1ps
// =============================================================
// KPU — Throughput-Optimised
// Paper: "Energy-Efficient and High-Throughput CNN Inference
//         Engine Based on Memory-Sharing and Data-Reusing
//         for Edge Applications", Islam et al., TCAS-I 2024
//
// Modules in this file
//   pkg_KPU        — central parameter package
//   KPC            — 4-state kernel processing controller  (§III-A, THR-1)
//   Line_Memory    — dual read-pointer, registered output buffer (§III-A.2, THR-2)
//   PE_core        — multi-purpose reconfigurable PE           (§III-A.1, THR-3)
//   adder_tree     — pipelined log₂M-stage binary reduction   (THR-5)
//   KPU_cluster    — top-level KPU wrapper                    (THR-4 / THR-6)
//
// Throughput optimisations (THR-x tags used throughout):
//   THR-1  KPC 4-state FSM ensures σ = 1 (no PE idle between strides).
//   THR-2  Dual RA pointers in Line_Memory for data reuse (§III-A.2).
//   THR-3  Single-multiplier critical path; IDM + Stride_Request flow.
//   THR-4  Line-selection mux rotated per vertical stride (Fig. 6).
//   THR-5  Pipelined adder tree replaces combinational reduction.
//   THR-6  K-wide serial weight bus replaces M×N×K broadcast.
// =============================================================


// =============================================================
// pkg_KPU — central parameter package
// =============================================================
package pkg_KPU;

    // PE array dimensions
    parameter int M         = 9;    // rows  (= filter height α for 3×3)
    parameter int N         = 96;   // cols  (parallelism per row)
    parameter int K         = 16;   // data / weight bit-width (Q8.8)

    // Derived bit-widths
    parameter int M_BW      = $clog2(M + 1);
    parameter int N_BW      = $clog2(N + 1);
    parameter int K2        = 2 * K;

    // Weight SRAM per PE  (z weights of k bits each)
    parameter int Z         = 16;
    parameter int Z_BW      = $clog2(Z + 1);

    // Line-memory size (≥ one row of input feature map)
    parameter int A         = 128;
    parameter int A_BW      = $clog2(A);

    // Stride size
    parameter int S         = 1;

    // Pre-fetch threshold r (Alg-1 L3)
    parameter int R_VAL     = 3;    // r = α for 3×3 kernel
    parameter int R_BW      = $clog2(R_VAL + 1);

    // AvgPool log₂ shift  ⌊log₂(α·β)⌋  (Alg-1 L4)
    parameter int AVG_SHIFT = 3;    // 3×3 → shift 3

    // Computation-type encoding
    typedef enum logic [2:0] {
        CONV    = 3'd0,
        FC      = 3'd1,
        MAXPOOL = 3'd2,
        AVGPOOL = 3'd3,
        RELU    = 3'd4,
        RELU6   = 3'd5
    } comp_t;

endpackage


// =============================================================
// KPC — Kernel Processing Controller  (THR-1)
//
// 4-state FSM:
//   LOAD        Stream Z weights into every PE (THR-6).
//   PREFETCH    Push r_min input items before any MAC (Alg-1 L5-13).
//   COMPUTE     MAC + simultaneous residual data fetch (Alg-1 L14-15).
//   STRIDE_NEXT Tail-overlap pre-fetch for the next vertical stride
//               (Alg-1 L16-22); immediately resumes COMPUTE, so PEs
//               are never idle → σ = 1.
// =============================================================
module KPC
import pkg_KPU::*;
#(
    parameter int M_ROWS = M,
    parameter int Z_FILT = Z,
    parameter int R_MIN  = R_VAL,
    parameter int A_SIZE = A,
    parameter int S_STEP = S
)
(
    input  logic                clk,
    input  logic                rst,

    // Layer configuration (from IEC)
    input  logic                layer_start,
    input  comp_t               comp_type,
    input  logic [Z_BW-1:0]    delta,
    input  logic [R_BW-1:0]    r_min,
    input  logic [A_BW-1:0]    A_size,
    input  logic [M_BW-1:0]    alpha,

    // Data-ready handshakes
    input  logic                dram_I_valid,
    input  logic                dram_W_valid,

    // Stride requests from col-0 PE of each row
    input  logic [M_ROWS-1:0]  stride_req,

    // Control to Line Memories
    output logic                Write_Sel,
    output logic                Read_Sel,
    output logic [M_ROWS-1:0]  Next_Stride,
    output logic [M_ROWS-1:0]  Reuse_Sel,

    // Control to PEs
    output logic [1:0]          Wr_Rr,         // [1]=WE  [0]=RE
    output logic [Z_BW-1:0]    WA_out,
    output logic                WE_out,

    // Line-selection for vertical stride routing (THR-4)
    output logic [M_BW-1:0]    line_sel [0:M_ROWS-1],

    // Status
    output logic                load_done,
    output logic                compute_active,
    output logic                layer_done
);
    typedef enum logic [1:0] {
        ST_LOAD     = 2'b00,
        ST_PREFETCH = 2'b01,
        ST_COMPUTE  = 2'b10,
        ST_STR_NEXT = 2'b11
    } kpc_state_t;

    kpc_state_t state;

    logic [Z_BW-1:0]  load_cnt;
    logic [R_BW-1:0]  pre_cnt;
    logic [A_BW-1:0]  hstride_cnt;
    logic [M_BW-1:0]  vstride_cnt;
    logic [M_BW-1:0]  lm_base;

    wire load_full = (load_cnt    == Z_BW'(Z_FILT - 1));
    wire pre_done  = (pre_cnt     >= R_BW'(R_MIN  - 1));
    wire h_full    = (hstride_cnt == A_BW'(A_size - 1));
    wire v_full    = (vstride_cnt == M_BW'(alpha  - 1));

    always_ff @(posedge clk) begin
        if (rst || layer_start) begin
            state         <= ST_LOAD;
            load_cnt      <= '0;
            pre_cnt       <= '0;
            hstride_cnt   <= '0;
            vstride_cnt   <= '0;
            lm_base       <= '0;
            load_done     <= 1'b0;
            layer_done    <= 1'b0;
        end else begin
            case (state)

                ST_LOAD: begin
                    if (dram_W_valid) begin
                        if (load_full) begin
                            load_done <= 1'b1;
                            load_cnt  <= '0;
                            pre_cnt   <= '0;
                            state     <= ST_PREFETCH;
                        end else
                            load_cnt <= load_cnt + 1'b1;
                    end
                end

                ST_PREFETCH: begin
                    if (dram_I_valid) begin
                        if (pre_done) begin
                            pre_cnt <= '0;
                            state   <= ST_COMPUTE;
                        end else
                            pre_cnt <= pre_cnt + 1'b1;
                    end
                end

                ST_COMPUTE: begin
                    if (stride_req[0]) begin
                        if (h_full) begin
                            hstride_cnt <= '0;
                            pre_cnt     <= '0;
                            state       <= ST_STR_NEXT;
                        end else
                            hstride_cnt <= hstride_cnt + 1'b1;
                    end
                end

                ST_STR_NEXT: begin
                    if (dram_I_valid) begin
                        if (pre_done) begin
                            if (v_full) begin
                                vstride_cnt <= '0;
                                layer_done  <= 1'b1;
                                state       <= ST_LOAD;
                            end else begin
                                vstride_cnt <= vstride_cnt + 1'b1;
                                lm_base <= (lm_base + M_BW'(S_STEP) >= M_BW'(M_ROWS))
                                           ? lm_base + M_BW'(S_STEP) - M_BW'(M_ROWS)
                                           : lm_base + M_BW'(S_STEP);
                                pre_cnt <= '0;
                                state   <= ST_COMPUTE;
                            end
                        end else
                            pre_cnt <= pre_cnt + 1'b1;
                    end
                end

            endcase
        end
    end

    assign Write_Sel      = (state == ST_LOAD);
    assign Read_Sel       = (state == ST_COMPUTE || state == ST_STR_NEXT);
    assign Wr_Rr          = {Write_Sel, Read_Sel};
    assign WE_out         = (state == ST_LOAD) && dram_W_valid;
    assign WA_out         = load_cnt;
    assign compute_active = (state == ST_COMPUTE);

    genvar r;
    generate
        for (r = 0; r < M_ROWS; r++) begin : NS_GEN
            assign Next_Stride[r] = compute_active && stride_req[r];
        end
    endgenerate

    // THR-4: line_sel rotation.
    // PE row i → LM index (lm_base + i) mod M_ROWS.
    // Top (alpha-s) rows carry reused data; bottom s rows carry fresh data.
    genvar ls;
    generate
        for (ls = 0; ls < M_ROWS; ls++) begin : LSEL
            wire [M_BW-1:0] raw = lm_base + M_BW'(ls);
            assign line_sel[ls]  = (raw >= M_BW'(M_ROWS)) ? raw - M_BW'(M_ROWS) : raw;
            assign Reuse_Sel[ls] = (ls < (M_ROWS - S_STEP));
        end
    endgenerate

endmodule


// =============================================================
// Line_Memory — dual read-pointer, registered output buffer  (THR-2)
//
// Matches §III-A.2 / Fig. 5:
//   k×A memory with independent WAG and RAG.
//   RA_new   : advances by S_STEP on each Next_Stride (fresh data).
//   RA_ruse  : replays the previous vertical stride's offset (reuse).
//   Reuse_Sel from KPC chooses the active pointer.
//   Output buffer: N registered k-bit FFs; S_STEP slots load from
//   memory on each stride; remaining N−S_STEP slots shift (horizontal
//   reuse).
// =============================================================
module Line_Memory
import pkg_KPU::*;
#(
    parameter int N_PE   = N,
    parameter int BIT_W  = K,
    parameter int A_SIZE = A,
    parameter int S_STEP = S
)
(
    input  logic                      clk,
    input  logic                      rst,

    input  logic signed [BIT_W-1:0]  I,
    input  logic                      Write_Sel,

    input  logic                      Read_Sel,
    input  logic                      Next_Stride,
    input  logic                      Reuse_Sel,

    output logic signed [BIT_W-1:0]  O [0:N_PE-1]
);
    localparam int ABITS = $clog2(A_SIZE);

    logic signed [BIT_W-1:0] mem [0:A_SIZE-1];
    logic [ABITS-1:0]         wag_addr;

    // WAG: sequential write, wraps at A_SIZE
    always_ff @(posedge clk) begin
        if (rst)
            wag_addr <= '0;
        else if (Write_Sel) begin
            mem[wag_addr] <= I;
            wag_addr <= (wag_addr == ABITS'(A_SIZE - 1)) ? '0 : wag_addr + 1'b1;
        end
    end

    // Dual RAG (THR-2)
    logic [ABITS-1:0] RA_new;
    logic [ABITS-1:0] RA_ruse;

    function automatic logic [ABITS-1:0] mod_add
        (input logic [ABITS-1:0] base, input int step, input int sz);
        logic [ABITS:0] tmp;
        tmp = {1'b0, base} + ABITS'(step);
        return (tmp >= ABITS'(sz)) ? tmp[ABITS-1:0] - ABITS'(sz) : tmp[ABITS-1:0];
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            RA_new  <= '0;
            RA_ruse <= '0;
        end else if (Read_Sel && Next_Stride) begin
            if (!Reuse_Sel)
                RA_new  <= mod_add(RA_new,  S_STEP, A_SIZE);
            else
                RA_ruse <= mod_add(RA_ruse, S_STEP, A_SIZE);
        end
    end

    wire [ABITS-1:0] ra_active = Reuse_Sel ? RA_ruse : RA_new;

    // Registered output buffer: S_STEP fresh slots; N_PE-S_STEP shifted
    genvar p;
    generate
        for (p = 0; p < N_PE; p++) begin : OUT_BUF
            always_ff @(posedge clk) begin
                if (rst) begin
                    O[p] <= '0;
                end else if (Read_Sel && Next_Stride) begin
                    if (p < S_STEP) begin
                        // Fresh slot: read from memory
                        logic [ABITS-1:0] addr;
                        addr = ra_active + ABITS'(p);
                        if (addr >= ABITS'(A_SIZE)) addr -= ABITS'(A_SIZE);
                        O[p] <= mem[addr];
                    end else begin
                        // Reuse slot: retain older data (horizontal reuse)
                        O[p] <= O[p - S_STEP];
                    end
                end
                // else: hold
            end
        end
    endgenerate

endmodule


// =============================================================
// PE_core — multi-purpose reconfigurable PE  (THR-3)
//
// Matches §III-A.1 / Fig. 4.
// Supports: Conv, FC, MaxPool, AvgPool, ReLU, ReLU6.
// Critical path = single multiplier only (Table I note).
// Weight SRAM z×k written serially from KPC (THR-6).
// IDM gates MAC reads until r_val weights pre-loaded.
// Stride_Request fires once per δ-cycle epoch.
// SZD energy gate overridden for layer-0 via S_Ovd.
// =============================================================
module PE_core
import pkg_KPU::*;
#(
    parameter int BIT_W   = K,
    parameter int Z_FILT  = Z,
    parameter int AVG_SHF = AVG_SHIFT
)
(
    input  logic                      clk,
    input  logic                      rst,

    input  comp_t                     comp_type,
    input  logic                      S_Ovd,
    input  logic [1:0]                Wr_Rr,         // [1]=WE  [0]=RE
    input  logic [Z_BW-1:0]          WA_in,
    input  logic                      WE_in,
    input  logic [Z_BW-1:0]          delta,
    input  logic [Z_BW-1:0]          r_val,

    input  logic signed [BIT_W-1:0]  I_sel,
    input  logic signed [BIT_W-1:0]  B_Psum,
    input  logic signed [BIT_W-1:0]  W_in,
    input  logic signed [BIT_W-1:0]  clip6,

    output logic signed [BIT_W-1:0]  Psum,
    output logic                      Stride_Request
);
    // Weight SRAM
    logic signed [BIT_W-1:0] wmem [0:Z_FILT-1];
    always_ff @(posedge clk)
        if (WE_in) wmem[WA_in] <= W_in;

    // AGU_Write — track loaded weight count for IDM
    logic [Z_BW-1:0] WA_local;
    wire              WE_local = Wr_Rr[1];
    always_ff @(posedge clk) begin
        if (rst)          WA_local <= '0;
        else if (WE_local)
            WA_local <= (WA_local == delta - 1'b1) ? '0 : WA_local + 1'b1;
    end

    // IDM — block reads until r_val weights pre-loaded (§III-A.1)
    wire IDM_ok = (WA_local >= r_val);

    // AGU_Read — cycle through weight addresses
    logic [Z_BW-1:0] RA;
    wire              RE = IDM_ok && Wr_Rr[0];

    assign Stride_Request = RE && (RA == delta - 1'b1);

    always_ff @(posedge clk) begin
        if (rst || Stride_Request) RA <= '0;
        else if (RE)               RA <= RA + 1'b1;
    end

    wire signed [BIT_W-1:0] W_rd = RE ? wmem[RA] : '0;

    // Input register — latches I_sel on each Stride_Request
    logic signed [BIT_W-1:0] input_reg;
    always_ff @(posedge clk) begin
        if (rst)             input_reg <= '0;
        else if (Stride_Request) input_reg <= I_sel;
    end

    // SZD — sign-and-zero detector (§III-A.1)
    wire SZD_skip = S_Ovd ? 1'b0 : (input_reg <= '0);
    wire signed [BIT_W-1:0] I_mac = SZD_skip ? '0 : input_reg;
    wire signed [BIT_W-1:0] W_mac = SZD_skip ? '0 : W_rd;

    // MAC — single-multiplier critical path
    wire signed [2*BIT_W-1:0] product = I_mac * W_mac;
    logic signed [BIT_W-1:0]  mpr;
    always_ff @(posedge clk) begin
        if (rst)     mpr <= '0;
        else if (RE) mpr <= product[2*BIT_W-1 : BIT_W];
    end
    wire signed [BIT_W-1:0] mac_sum = mpr + B_Psum;

    // ReLU6 clip
    wire signed [BIT_W-1:0] relu6_out = (mac_sum > clip6) ? clip6 : mac_sum;

    // AvgPool: arithmetic right-shift (Alg-1 L4)
    wire signed [BIT_W-1:0] avgpool_out = mac_sum >>> AVG_SHF;

    // MaxPool
    wire signed [BIT_W-1:0] max_out = (I_mac >= B_Psum) ? I_mac : B_Psum;

    // Output mux
    logic signed [BIT_W-1:0] pe_out;
    always_comb begin
        case (comp_type)
            CONV    : pe_out = mac_sum;
            FC      : pe_out = mac_sum;
            AVGPOOL : pe_out = avgpool_out;
            MAXPOOL : pe_out = max_out;
            RELU    : pe_out = (mac_sum < '0) ? '0 : mac_sum;
            RELU6   : pe_out = (mac_sum < '0) ? '0 : relu6_out;
            default : pe_out = mac_sum;
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst)     Psum <= '0;
        else if (RE) Psum <= pe_out;
    end

endmodule


// =============================================================
// adder_tree — pipelined binary reduction  (THR-5)
//
// log₂(INPUTS) pipeline stages; input zero-padded to next
// power of two.  conv_valid in KPU_cluster is delayed by
// TREE_LAT = $clog2(INPUTS) cycles to match.
// =============================================================
module adder_tree
#(
    parameter int INPUTS   = 9,
    parameter int IN_WIDTH = 20
)
(
    input  logic                         clk,
    input  logic                         rst,
    input  logic signed [IN_WIDTH-1:0]  in_vec  [0:INPUTS-1],
    output logic signed [IN_WIDTH-1:0]  sum_out
);
    localparam int STAGES = $clog2(INPUTS);
    localparam int PAD    = 1 << STAGES;

    logic signed [IN_WIDTH-1:0] stage_r [0:STAGES-1][0:PAD-1];

    // Zero-pad input
    logic signed [IN_WIDTH-1:0] in_padded [0:PAD-1];
    genvar k;
    generate
        for (k = 0; k < PAD; k++) begin : IPAD
            if (k < INPUTS) assign in_padded[k] = in_vec[k];
            else             assign in_padded[k] = '0;
        end
    endgenerate

    // Stage 0
    genvar n0;
    generate
        localparam int NODES0 = PAD >> 1;
        for (n0 = 0; n0 < NODES0; n0++) begin : STAGE0_NODE
            always_ff @(posedge clk) begin
                if (rst) stage_r[0][n0] <= '0;
                else     stage_r[0][n0] <= in_padded[2*n0] + in_padded[2*n0+1];
            end
        end
        for (n0 = NODES0; n0 < PAD; n0++) begin : STAGE0_PAD
            always_ff @(posedge clk) stage_r[0][n0] <= '0;
        end
    endgenerate

    // Stages 1 .. STAGES-1
    genvar s, n;
    generate
        for (s = 1; s < STAGES; s++) begin : STAGE_S
            localparam int NODES = PAD >> (s + 1);
            for (n = 0; n < NODES; n++) begin : NODE
                always_ff @(posedge clk) begin
                    if (rst) stage_r[s][n] <= '0;
                    else     stage_r[s][n] <= stage_r[s-1][2*n] + stage_r[s-1][2*n+1];
                end
            end
            for (n = NODES; n < PAD; n++) begin : PAD_UP
                always_ff @(posedge clk) stage_r[s][n] <= '0;
            end
        end
    endgenerate

    assign sum_out = stage_r[STAGES-1][0];

endmodule


// =============================================================
// KPU_cluster — top-level KPU wrapper  (THR-1 … THR-6)
//
// Instantiates:
//   1 × KPC
//   M × Line_Memory
//   M × N × PE_core
//   1 × adder_tree
//
// Vertical stride routing (THR-4): KPC's line_sel[i] selects
// which LM drives PE row i (Fig. 6 connection-swap).
// conv_valid is delayed by TREE_LAT cycles to match adder_tree.
// =============================================================
module KPU_cluster
import pkg_KPU::*;
#(
    parameter int M_ROWS  = M,
    parameter int N_COLS  = N,
    parameter int BIT_W   = K,
    parameter int Z_FILT  = Z,
    parameter int A_SIZE  = A,
    parameter int S_STEP  = S,
    parameter int R_MIN   = R_VAL,
    parameter int AVG_SHF = AVG_SHIFT
)
(
    input  logic                    clk,
    input  logic                    rst,

    input  logic                    layer_start,
    input  comp_t                   comp_type,
    input  logic [Z_BW-1:0]        delta,
    input  logic [R_BW-1:0]        r_min,
    input  logic [A_BW-1:0]        A_size,
    input  logic [M_BW-1:0]        alpha,

    input  logic signed [BIT_W-1:0] I_data,
    input  logic                    I_valid,
    input  logic signed [BIT_W-1:0] W_data,
    input  logic                    W_valid,
    input  logic signed [BIT_W-1:0] B_data [0:M_ROWS-1],
    input  logic                    B_valid,

    input  logic                    S_Ovd,
    input  logic signed [BIT_W-1:0] clip6,

    output logic signed [BIT_W + $clog2(M_ROWS+1) - 1 : 0] conv_out,
    output logic                    conv_valid,
    output logic                    layer_done
);
    localparam int OUT_W    = BIT_W + $clog2(M_ROWS + 1);
    localparam int TREE_LAT = $clog2(M_ROWS);

    // KPC
    logic                Write_Sel, Read_Sel;
    logic [1:0]          Wr_Rr;
    logic [M_ROWS-1:0]   Next_Stride, Reuse_Sel;
    logic [M_BW-1:0]     line_sel [0:M_ROWS-1];
    logic [Z_BW-1:0]     WA_kpc;
    logic                WE_kpc;
    logic                load_done_kpc;
    logic                compute_active;
    logic [M_ROWS-1:0]   stride_req_from_pe;

    KPC #(
        .M_ROWS(M_ROWS), .Z_FILT(Z_FILT), .R_MIN(R_MIN),
        .A_SIZE(A_SIZE),  .S_STEP(S_STEP)
    ) kpc_inst (
        .clk            (clk),
        .rst            (rst),
        .layer_start    (layer_start),
        .comp_type      (comp_type),
        .delta          (delta),
        .r_min          (r_min),
        .A_size         (A_size),
        .alpha          (alpha),
        .dram_I_valid   (I_valid),
        .dram_W_valid   (W_valid),
        .stride_req     (stride_req_from_pe),
        .Write_Sel      (Write_Sel),
        .Read_Sel       (Read_Sel),
        .Next_Stride    (Next_Stride),
        .Reuse_Sel      (Reuse_Sel),
        .Wr_Rr          (Wr_Rr),
        .WA_out         (WA_kpc),
        .WE_out         (WE_kpc),
        .line_sel       (line_sel),
        .load_done      (load_done_kpc),
        .compute_active (compute_active),
        .layer_done     (layer_done)
    );

    // Line Memory array — M rows, one LM per row (§III-A.2, Fig. 5)
    logic signed [BIT_W-1:0] lm_out [0:M_ROWS-1][0:N_COLS-1];

    genvar r;
    generate
        for (r = 0; r < M_ROWS; r++) begin : LM_ROW
            wire signed [BIT_W-1:0] lm_row_out [0:N_COLS-1];
            Line_Memory #(
                .N_PE(N_COLS), .BIT_W(BIT_W), .A_SIZE(A_SIZE), .S_STEP(S_STEP)
            ) lm_inst (
                .clk         (clk),
                .rst         (rst),
                .I           (I_data),
                .Write_Sel   (Write_Sel),
                .Read_Sel    (Read_Sel),
                .Next_Stride (Next_Stride[r]),
                .Reuse_Sel   (Reuse_Sel[r]),
                .O           (lm_row_out)
            );
            genvar cc;
            for (cc = 0; cc < N_COLS; cc++) begin : LM_COPY
                assign lm_out[r][cc] = lm_row_out[cc];
            end
        end
    endgenerate

    // THR-4: line-selection mux — route LM outputs to PE rows
    logic signed [BIT_W-1:0] pe_lm_in [0:M_ROWS-1][0:N_COLS-1];
    genvar ri, ci;
    generate
        for (ri = 0; ri < M_ROWS; ri++) begin : LMUX_ROW
            for (ci = 0; ci < N_COLS; ci++) begin : LMUX_COL
                assign pe_lm_in[ri][ci] = lm_out[line_sel[ri]][ci];
            end
        end
    endgenerate

    // PE array — M_ROWS × N_COLS
    logic signed [BIT_W-1:0] psum [0:M_ROWS-1][0:N_COLS-1];

    genvar i, j;
    generate
        for (i = 0; i < M_ROWS; i++) begin : PE_ROW
            for (j = 0; j < N_COLS; j++) begin : PE_COL
                wire signed [BIT_W-1:0] b_in;
                if (j == 0) assign b_in = B_data[i];
                else        assign b_in = psum[i][j-1];

                wire sr_w;
                if (j == 0) assign stride_req_from_pe[i] = sr_w;

                PE_core #(
                    .BIT_W(BIT_W), .Z_FILT(Z_FILT), .AVG_SHF(AVG_SHF)
                ) pe_inst (
                    .clk             (clk),
                    .rst             (rst),
                    .comp_type       (comp_type),
                    .S_Ovd           (S_Ovd),
                    .Wr_Rr           (Wr_Rr),
                    .WA_in           (WA_kpc),
                    .WE_in           (WE_kpc),
                    .delta           (delta),
                    .r_val           (Z_BW'(r_min)),
                    .I_sel           (pe_lm_in[i][j]),
                    .B_Psum          (b_in),
                    .W_in            (W_data),
                    .clip6           (clip6),
                    .Psum            (psum[i][j]),
                    .Stride_Request  (sr_w)
                );
            end
        end
    endgenerate

    // THR-5: pipelined adder tree across rows
    logic signed [OUT_W-1:0] row_psums [0:M_ROWS-1];
    genvar rr;
    generate
        for (rr = 0; rr < M_ROWS; rr++) begin : ROW_OUT
            assign row_psums[rr] = OUT_W'(signed'(psum[rr][N_COLS-1]));
        end
    endgenerate

    adder_tree #(.INPUTS(M_ROWS), .IN_WIDTH(OUT_W)) atree (
        .clk    (clk),
        .rst    (rst),
        .in_vec (row_psums),
        .sum_out(conv_out)
    );

    // Align conv_valid with adder_tree output
    logic [TREE_LAT:0] valid_sr;
    always_ff @(posedge clk) begin
        if (rst) valid_sr <= '0;
        else     valid_sr <= {valid_sr[TREE_LAT-1:0], compute_active};
    end
    assign conv_valid = valid_sr[TREE_LAT];

endmodule
