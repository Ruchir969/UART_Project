`timescale 1ns / 1ps
module uart_rx #(
    parameter CLK_FREQ   = 50_000_000,
    parameter BAUD_RATE  = 115200,
    parameter FIFO_DEPTH = 8
)(
    input  wire        clk,
    input  wire        rst_n,

    // Configuration
    input  wire [15:0] baud_div,         // Same divisor as TX (÷16 used internally)
    input  wire [1:0]  parity_cfg,       // 00=none, 01=odd, 10=even
    input  wire        stop_bits,        // 0=1 stop bit, 1=2 stop bits
    input  wire        data_bits_cfg,    // 0=8 bits, 1=7 bits

    // Serial input
    input  wire        rx,               // UART RX line

    // Read interface
    input  wire        rd_en,            // Read enable (pulse to pop byte from FIFO)
    output wire [7:0]  rd_data,          // Received data byte
    output wire        rx_data_valid,    // High when FIFO has data

    // Status / Error flags
    output reg         parity_err,       // Parity mismatch detected
    output reg         frame_err,        // Stop bit was not HIGH
    output reg         overrun_err,      // New byte received before FIFO was read
    output wire        rx_fifo_full,
    output wire        rx_fifo_empty,
    output wire [3:0]  rx_fifo_count
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
// RX input synchroniser (2-FF metastability guard)
// ---------------------------------------------------------------------------
reg rx_sync1, rx_sync2, rx_sync3;
always @(posedge clk) begin
    if (!rst_n) begin
        rx_sync1 <= 1'b1;
        rx_sync2 <= 1'b1;
        rx_sync3 <= 1'b1;
    end else begin
        rx_sync1 <= rx;
        rx_sync2 <= rx_sync1;
        rx_sync3 <= rx_sync2;
    end
end
// Majority vote across 3 pipeline stages — reduces glitch sensitivity
wire rx_clean = (rx_sync1 & rx_sync2) | (rx_sync2 & rx_sync3) | (rx_sync1 & rx_sync3);

// ---------------------------------------------------------------------------
// 16x oversampling baud tick
// ---------------------------------------------------------------------------
reg [15:0] os_cnt;
reg        os_tick;          // fires 16x per bit period

wire [15:0] os_div = (baud_div >> 4);  // divide by 16

always @(posedge clk) begin
    if (!rst_n) begin
        os_cnt  <= 0;
        os_tick <= 1'b0;
    end else begin
        if (os_cnt == os_div) begin
            os_cnt  <= 0;
            os_tick <= 1'b1;
        end else begin
            os_cnt  <= os_cnt + 1;
            os_tick <= 1'b0;
        end
    end
end

// ---------------------------------------------------------------------------
// RX FIFO
// ---------------------------------------------------------------------------
reg [7:0]           fifo_mem [0:FIFO_DEPTH-1];
reg [FIFO_PTR_W:0]  fifo_wr_ptr;
reg [FIFO_PTR_W:0]  fifo_rd_ptr;

wire [FIFO_PTR_W:0] fifo_count_w = fifo_wr_ptr - fifo_rd_ptr;
assign rx_fifo_count = fifo_count_w[3:0];
assign rx_fifo_full  = (fifo_count_w == FIFO_DEPTH);
assign rx_fifo_empty = (fifo_count_w == 0);
assign rx_data_valid = !rx_fifo_empty;
assign rd_data       = fifo_mem[fifo_rd_ptr[FIFO_PTR_W-1:0]];

// FIFO read (destructive pop)
always @(posedge clk) begin
    if (!rst_n)
        fifo_rd_ptr <= 0;
    else if (rd_en && !rx_fifo_empty)
        fifo_rd_ptr <= fifo_rd_ptr + 1;
end

// ---------------------------------------------------------------------------
// RX FSM with 16x oversampling counter
// ---------------------------------------------------------------------------
reg [2:0]  state;
reg [3:0]  os_phase;     // counts 0..15 within each bit
reg [3:0]  bit_cnt;
reg [7:0]  shift_reg;
reg        parity_calc;

always @(posedge clk) begin
    if (!rst_n) begin
        state        <= ST_IDLE;
        os_phase     <= 0;
        bit_cnt      <= 0;
        shift_reg    <= 8'h00;
        parity_calc  <= 1'b0;
        parity_err   <= 1'b0;
        frame_err    <= 1'b0;
        overrun_err  <= 1'b0;
        fifo_wr_ptr  <= 0;
    end else begin
        // Clear single-cycle error pulses
        parity_err  <= 1'b0;
        frame_err   <= 1'b0;
        overrun_err <= 1'b0;

        case (state)
            // -----------------------------------------------------------------
            ST_IDLE: begin
                if (rx_clean == 1'b0) begin  // Falling edge detected (start bit)
                    os_phase    <= 0;
                    parity_calc <= (parity_cfg == PAR_ODD) ? 1'b1 : 1'b0;
                    state       <= ST_START;
                end
            end

            // -----------------------------------------------------------------
            ST_START: begin
                if (os_tick) begin
                    if (os_phase == 4'd7) begin
                        // Sample at middle of start bit
                        if (rx_clean == 1'b0) begin
                            os_phase <= 0;
                            bit_cnt  <= 0;
                            state    <= ST_DATA;
                        end else begin
                            state <= ST_IDLE;  // False start — abort
                        end
                    end else begin
                        os_phase <= os_phase + 1;
                    end
                end
            end

            // -----------------------------------------------------------------
            ST_DATA: begin
                if (os_tick) begin
                    if (os_phase == 4'd15) begin
                        // Sample at center of each data bit (phase 15 = full bit elapsed)
                        os_phase    <= 0;
                        shift_reg   <= {rx_clean, shift_reg[7:1]};  // LSB first
                        parity_calc <= parity_calc ^ rx_clean;
                        if (bit_cnt == (data_bits_cfg ? 3'd6 : 3'd7)) begin
                            bit_cnt <= 0;
                            state   <= (parity_cfg == PAR_NONE) ? ST_STOP1 : ST_PAR;
                        end else begin
                            bit_cnt <= bit_cnt + 1;
                        end
                    end else begin
                        os_phase <= os_phase + 1;
                    end
                end
            end

            // -----------------------------------------------------------------
            ST_PAR: begin
                if (os_tick) begin
                    if (os_phase == 4'd15) begin
                        os_phase <= 0;
                        if (rx_clean != parity_calc)
                            parity_err <= 1'b1;
                        state <= ST_STOP1;
                    end else begin
                        os_phase <= os_phase + 1;
                    end
                end
            end

            // -----------------------------------------------------------------
            ST_STOP1: begin
                if (os_tick) begin
                    if (os_phase == 4'd15) begin
                        os_phase <= 0;
                        if (rx_clean != 1'b1) begin
                            frame_err <= 1'b1;
                            state     <= ST_IDLE;
                        end else begin
                            // Push received byte into FIFO
                            if (!rx_fifo_full) begin
                                fifo_mem[fifo_wr_ptr[FIFO_PTR_W-1:0]] <= shift_reg;
                                fifo_wr_ptr <= fifo_wr_ptr + 1;
                            end else begin
                                overrun_err <= 1'b1;
                            end
                            state <= stop_bits ? ST_STOP2 : ST_IDLE;
                        end
                    end else begin
                        os_phase <= os_phase + 1;
                    end
                end
            end

            // -----------------------------------------------------------------
            ST_STOP2: begin
                if (os_tick) begin
                    if (os_phase == 4'd15) begin
                        os_phase <= 0;
                        if (rx_clean != 1'b1)
                            frame_err <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        os_phase <= os_phase + 1;
                    end
                end
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
