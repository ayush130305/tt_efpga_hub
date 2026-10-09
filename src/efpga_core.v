// efpga_core.v
// Small hybrid eFPGA: NC registered LUT4+FF cells, three hard blocks (shift8, div8, crc8),
// windowed routing muxes, and one scan-chain bitstream.
// Every routing mux sees only a WINDOW of the source vector, so a design has to be placed
// with that locality in mind (test/cfgimage.py mirrors these windows exactly).

// Source-select mux: picks one of WIN candidates taken from a wrapped region of the source vector.
module efpga_mux #(
    parameter NS   = 16, // total sources in the source vector
    parameter R    = 0,  // first index of the region this mux draws from
    parameter L    = 16, // length of that region
    parameter OFF  = 0,  // starting offset inside the region (already reduced mod L)
    parameter WIN  = 8,  // number of candidates (<= L)
    parameter SELW = 3   // select width, clog2(WIN)
) (
    input  wire [NS-1:0]   src, // full source vector
    input  wire [SELW-1:0] sel, // select bits from the config chain
    output wire            y    // selected source bit
);
    // Number of mux leaves, padded up to a power of two.
    localparam P = (1 << SELW);

    // Candidate vector, zero-padded beyond WIN.
    wire [P-1:0] cand;

    // Generate variable for the candidate loop.
    genvar k;

    // This generate builds the candidate list from the wrapped region window.
    generate
        for (k = 0; k < P; k = k + 1) begin : g_cand
            if (k < WIN) begin : g_real
                // Real candidate: region start plus wrapped offset.
                assign cand[k] = src[R + ((OFF + k) % L)];
            end else begin : g_pad
                // Padding candidate for unused select codes.
                assign cand[k] = 1'b0;
            end
        end
    endgenerate

    // Final selection.
    assign y = cand[sel];
endmodule

// Hard block: 8-bit shift register with parallel load, used as the SPI/UART data shifter.
module hb_shift8 (
    input  wire       clk,   // system clock
    input  wire       rst_n, // active-low synchronous reset
    input  wire       sin,   // serial input shifted into the LSB
    input  wire       sh_en, // shift enable
    input  wire       ld,    // parallel load enable
    input  wire [7:0] pin,   // parallel load data
    output wire       msb,   // serial output (MSB)
    output wire [7:0] par    // parallel contents of the register
);
    // Shift register storage.
    reg [7:0] sh;

    // This always block holds the shift register: reset, load, shift.
    always @(posedge clk) begin
        // Reset clears the register.
        if (!rst_n) sh <= 8'd0;
        // Load takes priority over shift.
        else if (ld) sh <= pin;
        // Shift left, taking the serial input into bit 0.
        else if (sh_en) sh <= {sh[6:0], sin};
    end

    // Serial output is the MSB.
    assign msb = sh[7];
    // Parallel output is the whole register.
    assign par = sh;
endmodule

// Hard block: 8-bit programmable down-counter divider that emits a one-cycle tick.
module hb_div8 (
    input  wire       clk,    // system clock
    input  wire       rst_n,  // active-low synchronous reset
    input  wire       en,     // count enable (low reloads the counter)
    input  wire [7:0] reload, // reload value from the config chain
    output reg        tick    // registered one-cycle tick at terminal count
);
    // Down-counter storage.
    reg [7:0] cnt;

    // This always block runs the divider: reset, hold-in-reload, count, terminal reload.
    always @(posedge clk) begin
        // Reset clears counter and tick.
        if (!rst_n) begin
            cnt  <= 8'd0;
            tick <= 1'b0;
        // When disabled, keep the counter at the reload value and tick low.
        end else if (!en) begin
            cnt  <= reload;
            tick <= 1'b0;
        // At terminal count, reload and pulse tick.
        end else if (cnt == 8'd0) begin
            cnt  <= reload;
            tick <= 1'b1;
        // Otherwise count down.
        end else begin
            cnt  <= cnt - 8'd1;
            tick <= 1'b0;
        end
    end
endmodule

// Hard block: 8-bit serial CRC LFSR with a polynomial taken from the config chain.
module hb_crc8 (
    input  wire       clk,   // system clock
    input  wire       rst_n, // active-low synchronous reset
    input  wire       sin,   // serial data bit in
    input  wire       en,    // update enable
    input  wire       clr,   // clear the CRC register
    input  wire [7:0] poly,  // polynomial taps from the config chain
    output wire       msb    // CRC MSB
);
    // CRC register storage.
    reg [7:0] crc;

    // Feedback bit: MSB xor incoming data.
    wire fb = crc[7] ^ sin;

    // This always block updates the CRC: reset/clear, then shift with conditional polynomial xor.
    always @(posedge clk) begin
        // Reset or clear zeroes the register.
        if (!rst_n || clr) crc <= 8'd0;
        // Shift left and xor in the polynomial when feedback is set.
        else if (en) crc <= {crc[6:0], 1'b0} ^ (fb ? poly : 8'd0);
    end

    // Serial output is the MSB.
    assign msb = crc[7];
endmodule

// Fabric top.
//   Source vector S (index order, shared by cfgimage.py):
//     0 const0, 1 const1, 2..9 ui_in[7:0], 10..13 hub events,
//     14..14+NC-1 cell outputs, then shift-msb, div-tick, crc-msb.
module efpga_core #(
    parameter NC       = 16, // number of LUT4+FF cells (>= W)
    parameter W        = 8,  // candidates per cell-input / hard-block-sink mux
    parameter WO       = 8,  // candidates per output mux
    parameter CFG_MODE = 0   // 0: shift on clk using a synchronised cfg_clk edge, 1: shift on cfg_clk itself
) (
    input  wire       clk,        // system clock
    input  wire       rst_n,      // active-low reset
    input  wire [7:0] ui_in,      // dedicated input pins
    input  wire [3:0] hub_ev,     // hub events: rx_valid, spi_done, i2c_done, uart_tx_start
    input  wire       cfg_data,   // config bit in
    input  wire       cfg_clk,    // config shift clock (slow, see README)
    output wire       cfg_out,    // config chain tail, for read-back
    output wire [6:0] fab_out,    // routed fabric outputs
    output wire [6:0] sh_par_out, // shift register parallel byte (low 7 bits)
    output wire       uo_mode     // 1: shift register byte replaces fab_out on the pins
);
    // Ceiling log2 helper.
    function integer clog2(input integer v);
        integer n;
        begin
            clog2 = 0;
            for (n = v - 1; n > 0; n = n >> 1) clog2 = clog2 + 1;
        end
    endfunction

    // Pin-side sources: 8 ui_in plus 4 hub events.
    localparam NI    = 12;
    // Hard-block output sources (shift msb, div tick, crc msb).
    localparam NH    = 3;
    // First cell index in the source vector.
    localparam CB    = 2 + NI;
    // Total sources.
    localparam NS    = CB + NC + NH;
    // Fabric output pins.
    localparam NOUT  = 7;
    // Hard-block control sinks routed from the fabric.
    localparam NSINK = 7;
    // Output mux region: cells plus hard-block outputs.
    localparam LO    = NC + NH;
    // Effective windows.
    localparam WE    = (W  > NC) ? NC : W;
    localparam WOE   = (WO > LO) ? LO : WO;
    // Select widths.
    localparam SW    = clog2(WE);
    localparam SWO   = clog2(WOE);
    // Config bits per cell: 16 LUT bits + 4 input selects.
    localparam CELLB = 16 + 4 * SW;
    // Config chain layout offsets.
    localparam OFF_SINK = NC * CELLB;
    localparam OFF_DIV  = OFF_SINK + NSINK * SW;
    localparam OFF_CRC  = OFF_DIV + 8;
    localparam OFF_OUT  = OFF_CRC + 8;
    localparam OFF_UOM  = OFF_OUT + NOUT * SWO;
    localparam NCFG     = OFF_UOM + 1;

    // Config shift register (the whole bitstream, first bit in ends at the MSB).
    reg [NCFG-1:0] cfg;

    // Config chain tail for read-back.
    assign cfg_out = cfg[NCFG-1];

    // Generate selection of the config shift clocking scheme.
    generate
        if (CFG_MODE == 1) begin : g_cfg_pin
            // This always block shifts the bitstream on every cfg_clk rising edge.
            always @(posedge cfg_clk) begin
                // Shift left, taking cfg_data into bit 0.
                cfg <= {cfg[NCFG-2:0], cfg_data};
            end
        end else begin : g_cfg_sync
            // Synchroniser stages for cfg_clk and the data bit travelling with it.
            reg c1, c2, c3, d1, d2;
            // This always block synchronises the two config pins into the clk domain.
            always @(posedge clk) begin
                // Two-flop synchroniser for the clock pin, plus a delayed copy for edge detect.
                c1 <= cfg_clk; c2 <= c1; c3 <= c2;
                // Data goes through the same depth so it stays aligned with the clock edge.
                d1 <= cfg_data; d2 <= d1;
            end
            // This always block shifts one bit on each detected cfg_clk rising edge.
            always @(posedge clk) begin
                // Shift left on a synchronised rising edge.
                if (c2 & ~c3) cfg <= {cfg[NCFG-2:0], d2};
            end
        end
    endgenerate

    // Source vector shared by every routing mux.
    wire [NS-1:0] S;
    // Registered outputs of the logic cells.
    wire [NC-1:0] cell_q;
    // Hard block outputs.
    wire hb_sh_msb, hb_div_tick, hb_crc_msb;
    wire [7:0] hb_sh_par;

    // Source ordering: const0, const1, pins, hub events, cell outputs, hard-block outputs.
    assign S = {hb_crc_msb, hb_div_tick, hb_sh_msb, cell_q, hub_ev, ui_in, 1'b1, 1'b0};

    // Generate variables.
    genvar i, j;

    // This generate builds all logic cells.
    generate
        for (i = 0; i < NC; i = i + 1) begin : g_cell
            // LUT truth table from the config chain.
            wire [15:0] lut = cfg[i*CELLB +: 16];
            // The four LUT input bits after routing.
            wire [3:0] lin;
            // Input 0: local cells, window starts 3 below this cell.
            efpga_mux #(.NS(NS), .R(CB), .L(NC), .OFF((i - 3 + 2*NC) % NC), .WIN(WE), .SELW(SW)) u_m0 (
                .src(S), .sel(cfg[i*CELLB + 16 + 0*SW +: SW]), .y(lin[0]));
            // Input 1: backward cells, window ends at this cell.
            efpga_mux #(.NS(NS), .R(CB), .L(NC), .OFF((i - WE + 1 + 2*NC) % NC), .WIN(WE), .SELW(SW)) u_m1 (
                .src(S), .sel(cfg[i*CELLB + 16 + 1*SW +: SW]), .y(lin[1]));
            // Input 2: forward cells, window starts at this cell.
            efpga_mux #(.NS(NS), .R(CB), .L(NC), .OFF(i % NC), .WIN(WE), .SELW(SW)) u_m2 (
                .src(S), .sel(cfg[i*CELLB + 16 + 2*SW +: SW]), .y(lin[2]));
            // Input 3: whole source vector (constants, pins, events, cells, hard blocks).
            efpga_mux #(.NS(NS), .R(0), .L(NS), .OFF((2*i) % NS), .WIN(WE), .SELW(SW)) u_m3 (
                .src(S), .sel(cfg[i*CELLB + 16 + 3*SW +: SW]), .y(lin[3]));
            // Registered LUT output.
            reg q;
            // This always block registers the LUT output with a synchronous reset.
            always @(posedge clk) begin
                // Reset clears the cell.
                if (!rst_n) q <= 1'b0;
                // Otherwise capture the LUT output.
                else q <= lut[lin];
            end
            // Publish the cell output to the source vector.
            assign cell_q[i] = q;
        end
    endgenerate

    // Routed control signals for the hard blocks.
    wire [NSINK-1:0] sink;

    // This generate builds the routing muxes for the hard-block control inputs.
    generate
        for (j = 0; j < NSINK; j = j + 1) begin : g_sink
            // Sink windows are spread over the whole source vector.
            efpga_mux #(.NS(NS), .R(0), .L(NS), .OFF((4*j + 1) % NS), .WIN(WE), .SELW(SW)) u_m (
                .src(S), .sel(cfg[OFF_SINK + j*SW +: SW]), .y(sink[j]));
        end
    endgenerate

    // Shift register hard block.
    hb_shift8 u_shift (
        .clk(clk), .rst_n(rst_n),
        .sin(sink[0]), .sh_en(sink[1]), .ld(sink[2]),
        .pin(ui_in),
        .msb(hb_sh_msb), .par(hb_sh_par)
    );

    // Divider hard block.
    hb_div8 u_div (
        .clk(clk), .rst_n(rst_n),
        .en(sink[3]), .reload(cfg[OFF_DIV +: 8]),
        .tick(hb_div_tick)
    );

    // CRC hard block.
    hb_crc8 u_crc (
        .clk(clk), .rst_n(rst_n),
        .sin(sink[4]), .en(sink[5]), .clr(sink[6]),
        .poly(cfg[OFF_CRC +: 8]),
        .msb(hb_crc_msb)
    );

    // This generate builds the output-pin routing muxes (cells and hard blocks only).
    generate
        for (j = 0; j < NOUT; j = j + 1) begin : g_out
            // Output windows are spread over the cell/hard-block region.
            efpga_mux #(.NS(NS), .R(CB), .L(LO), .OFF((3*j) % LO), .WIN(WOE), .SELW(SWO)) u_m (
                .src(S), .sel(cfg[OFF_OUT + j*SWO +: SWO]), .y(fab_out[j]));
        end
    endgenerate

    // Pin-mode bit and shift register byte go to the top level for the pin mux.
    assign uo_mode    = cfg[OFF_UOM];
    assign sh_par_out = hb_sh_par[6:0];

    // Unused upper bit of the shift register parallel byte.
    wire _unused = &{hb_sh_par[7], 1'b0};
endmodule
