`timescale 1ns / 1ps
// =============================================================================
// Module      : uart_tx.v
// Description : Enhanced UART Transmitter
//               - Configurable baud rate via programmable divisor
//               - 8-deep transmit FIFO
//               - Parity support (none / even / odd)
//               - Configurable data bits (7 or 8)
//               - 1 or 2 stop bits
//               - tx_busy and tx_fifo_full status flags
//               - Oversampling clock for clean edges
// Author      : Enhanced UART Project
// =============================================================================

module uart_tx #(
    parameter CLK_FREQ    = 50_000_000,  // System clock frequency (Hz)
    parameter BAUD_RATE   = 115200,      // Default baud rate
    parameter FIFO_DEPTH  = 8            // TX FIFO depth (must be power of 2)
)(
    input  wire        clk,
    input  wire        rst_n,            // Active-low synchronous reset

    // Configuration inputs (can be tied to registers)
    input  wire [15:0] baud_div,         // Baud divisor: CLK_FREQ / BAUD_RATE - 1
    input  wire [1:0]  parity_cfg,       // 00=none, 01=odd, 10=even
    input  wire        stop_bits,        // 0=1 stop bit, 1=2 stop bits
    input  wire        data_bits_cfg,    // 0=8 bits, 1=7 bits

    // Write interface
    input  wire        wr_en,            // Write enable (pulse to push byte into FIFO)
    input  wire [7:0]  wr_data,          // Data byte to transmit

    // Serial output
    output reg         tx,               // UART TX line

    // Status
    output wire        tx_busy,          // High when transmitting
    output wire        tx_fifo_full,     // High when FIFO is full
    output wire        tx_fifo_empty,    // High when FIFO is empty
    output wire [3:0]  tx_fifo_count     // Number of bytes in FIFO
);

// ---------------------------------------------------------------------------
// Local parameters
// ---------------------------------------------------------------------------
localparam FIFO_PTR_W = $clog2(FIFO_DEPTH);

localparam [2:0]
    ST_IDLE  = 3'd0,
    ST_START = 3'd1,
    ST_DATA  = 3'd2,
    ST_PAR   = 3'd3,
    ST_STOP1 = 3'd4,
    ST_STOP2 = 3'd5;

localparam PAR_NONE = 2'b00,
           PAR_ODD  = 2'b01,
           PAR_EVEN = 2'b10;

// ---------------------------------------------------------------------------
// TX FIFO (synchronous, 8 x 8-bit)
// ---------------------------------------------------------------------------
reg [7:0]           fifo_mem [0:FIFO_DEPTH-1];
reg [FIFO_PTR_W:0]  fifo_wr_ptr;
reg [FIFO_PTR_W:0]  fifo_rd_ptr;

wire [FIFO_PTR_W:0] fifo_count_w = fifo_wr_ptr - fifo_rd_ptr;
assign tx_fifo_count = fifo_count_w[3:0];
assign tx_fifo_full  = (fifo_count_w == FIFO_DEPTH);
assign tx_fifo_empty = (fifo_count_w == 0);

// FIFO write
always @(posedge clk) begin
    if (!rst_n) begin
        fifo_wr_ptr <= 0;
    end else if (wr_en && !tx_fifo_full) begin
        fifo_mem[fifo_wr_ptr[FIFO_PTR_W-1:0]] <= wr_data;
        fifo_wr_ptr <= fifo_wr_ptr + 1;
    end
end

// ---------------------------------------------------------------------------
// Baud rate generator
// ---------------------------------------------------------------------------
reg [15:0] baud_cnt;
reg        baud_tick;

always @(posedge clk) begin
    if (!rst_n) begin
        baud_cnt  <= 0;
        baud_tick <= 1'b0;
    end else begin
        if (baud_cnt == baud_div) begin
            baud_cnt  <= 0;
            baud_tick <= 1'b1;
        end else begin
            baud_cnt  <= baud_cnt + 1;
            baud_tick <= 1'b0;
        end
    end
end

// ---------------------------------------------------------------------------
// TX FSM
// ---------------------------------------------------------------------------
reg [2:0]  state;
reg [7:0]  shift_reg;
reg [3:0]  bit_cnt;
reg        parity_bit;
reg        tx_active;

assign tx_busy = tx_active;

always @(posedge clk) begin
    if (!rst_n) begin
        state        <= ST_IDLE;
        tx           <= 1'b1;
        tx_active    <= 1'b0;
        fifo_rd_ptr  <= 0;
        shift_reg    <= 8'h00;
        bit_cnt      <= 0;
        parity_bit   <= 1'b0;
    end else begin
        case (state)
            // -----------------------------------------------------------------
            ST_IDLE: begin
                tx        <= 1'b1;
                tx_active <= 1'b0;
                if (!tx_fifo_empty && baud_tick) begin
                    // Load byte from FIFO
                    shift_reg   <= fifo_mem[fifo_rd_ptr[FIFO_PTR_W-1:0]];
                    fifo_rd_ptr <= fifo_rd_ptr + 1;
                    tx_active   <= 1'b1;
                    parity_bit  <= (parity_cfg == PAR_ODD) ? 1'b1 : 1'b0;
                    bit_cnt     <= 0;
                    state       <= ST_START;
                end
            end

            // -----------------------------------------------------------------
            ST_START: begin
                if (baud_tick) begin
                    tx    <= 1'b0;          // Start bit
                    state <= ST_DATA;
                end
            end

            // -----------------------------------------------------------------
            ST_DATA: begin
                if (baud_tick) begin
                    tx         <= shift_reg[0];
                    parity_bit <= parity_bit ^ shift_reg[0];
                    shift_reg  <= {1'b1, shift_reg[7:1]};  // LSB first
                    if (bit_cnt == (data_bits_cfg ? 3'd6 : 3'd7)) begin
                        bit_cnt <= 0;
                        state   <= (parity_cfg == PAR_NONE) ? ST_STOP1 : ST_PAR;
                    end else begin
                        bit_cnt <= bit_cnt + 1;
                    end
                end
            end

            // -----------------------------------------------------------------
            ST_PAR: begin
                if (baud_tick) begin
                    tx    <= parity_bit;
                    state <= ST_STOP1;
                end
            end

            // -----------------------------------------------------------------
            ST_STOP1: begin
                if (baud_tick) begin
                    tx    <= 1'b1;
                    state <= stop_bits ? ST_STOP2 : ST_IDLE;
                end
            end

            // -----------------------------------------------------------------
            ST_STOP2: begin
                if (baud_tick) begin
                    tx    <= 1'b1;
                    state <= ST_IDLE;
                end
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule