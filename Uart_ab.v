`timescale 1ns / 1ps
// =============================================================================
// Module      : uart_apb.v
// Description : APB3 Slave Wrapper for UART TX + RX
//               Exposes a clean register map a CPU/SoC can write to.
//
// Register Map (32-bit word-addressed):
//   0x00  CTRL     [0]=TX_EN, [1]=RX_EN, [2]=PARITY[1:0](bits3:2), [4]=STOP_BITS, [5]=DATA_BITS
//   0x04  BAUD     [15:0]=BAUD_DIV  (CLK_FREQ/BAUD_RATE - 1)
//   0x08  TXDATA   [7:0]=write byte into TX FIFO (write-only)
//   0x0C  RXDATA   [7:0]=read byte from RX FIFO  (read-only)
//   0x10  STATUS   [0]=TX_BUSY, [1]=TX_FIFO_FULL, [2]=TX_FIFO_EMPTY,
//                  [3]=RX_DATA_VALID, [4]=RX_FIFO_FULL, [5]=RX_FIFO_EMPTY,
//                  [6]=PARITY_ERR, [7]=FRAME_ERR, [8]=OVERRUN_ERR
//   0x14  FIFO_LVL [3:0]=TX_COUNT, [7:4]=RX_COUNT
//   0x18  IRQ_EN   [0]=TX_EMPTY_IE, [1]=RX_VALID_IE, [2]=ERROR_IE
//   0x1C  IRQ_STAT [0]=TX_EMPTY_IF, [1]=RX_VALID_IF, [2]=ERROR_IF (W1C)
//
// =============================================================================

module uart_apb #(
    parameter CLK_FREQ   = 50_000_000,
    parameter BAUD_RATE  = 115200,
    parameter FIFO_DEPTH = 8
)(
    // APB interface
    input  wire        PCLK,
    input  wire        PRESETn,
    input  wire [7:0]  PADDR,
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [31:0] PWDATA,
    output reg  [31:0] PRDATA,
    output wire        PREADY,
    output wire        PSLVERR,

    // UART pins
    input  wire        uart_rx_pin,
    output wire        uart_tx_pin,

    // Interrupt
    output wire        irq
);

// ---------------------------------------------------------------------------
// APB is always ready, no error
// ---------------------------------------------------------------------------
assign PREADY  = 1'b1;
assign PSLVERR = 1'b0;

// APB write strobe: only in ACCESS phase
wire apb_wr = PSEL & PENABLE & PWRITE;
wire apb_rd = PSEL & PENABLE & ~PWRITE;

// ---------------------------------------------------------------------------
// Configuration registers
// ---------------------------------------------------------------------------
reg [31:0] reg_ctrl;     // 0x00
reg [15:0] reg_baud;     // 0x04
reg [2:0]  reg_irq_en;   // 0x18
reg [2:0]  reg_irq_stat; // 0x1C

wire [1:0] parity_cfg    = reg_ctrl[3:2];
wire       stop_bits_cfg = reg_ctrl[4];
wire       data_bits_cfg = reg_ctrl[5];

// ---------------------------------------------------------------------------
// TX / RX wires
// ---------------------------------------------------------------------------
wire        tx_wr_en;
wire [7:0]  tx_wr_data;
wire        tx_busy, tx_fifo_full, tx_fifo_empty;
wire [3:0]  tx_fifo_count;

wire        rx_rd_en;
wire [7:0]  rx_rd_data;
wire        rx_data_valid, rx_fifo_full, rx_fifo_empty;
wire [3:0]  rx_fifo_count;
wire        parity_err, frame_err, overrun_err;

// Write to TXDATA register triggers FIFO push
assign tx_wr_en   = apb_wr && (PADDR[7:2] == 6'h02);  // addr 0x08
assign tx_wr_data = PWDATA[7:0];

// Read of RXDATA register triggers FIFO pop
assign rx_rd_en   = apb_rd && (PADDR[7:2] == 6'h03);  // addr 0x0C

// ---------------------------------------------------------------------------
// UART TX instance
// ---------------------------------------------------------------------------
uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE), .FIFO_DEPTH(FIFO_DEPTH)) u_tx (
    .clk            (PCLK),
    .rst_n          (PRESETn),
    .baud_div       (reg_baud),
    .parity_cfg     (parity_cfg),
    .stop_bits      (stop_bits_cfg),
    .data_bits_cfg  (data_bits_cfg),
    .wr_en          (tx_wr_en),
    .wr_data        (tx_wr_data),
    .tx             (uart_tx_pin),
    .tx_busy        (tx_busy),
    .tx_fifo_full   (tx_fifo_full),
    .tx_fifo_empty  (tx_fifo_empty),
    .tx_fifo_count  (tx_fifo_count)
);

// ---------------------------------------------------------------------------
// UART RX instance
// ---------------------------------------------------------------------------
uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE), .FIFO_DEPTH(FIFO_DEPTH)) u_rx (
    .clk            (PCLK),
    .rst_n          (PRESETn),
    .baud_div       (reg_baud),
    .parity_cfg     (parity_cfg),
    .stop_bits      (stop_bits_cfg),
    .data_bits_cfg  (data_bits_cfg),
    .rx             (uart_rx_pin),
    .rd_en          (rx_rd_en),
    .rd_data        (rx_rd_data),
    .rx_data_valid  (rx_data_valid),
    .parity_err     (parity_err),
    .frame_err      (frame_err),
    .overrun_err    (overrun_err),
    .rx_fifo_full   (rx_fifo_full),
    .rx_fifo_empty  (rx_fifo_empty),
    .rx_fifo_count  (rx_fifo_count)
);

// ---------------------------------------------------------------------------
// APB write logic
// ---------------------------------------------------------------------------
always @(posedge PCLK) begin
    if (!PRESETn) begin
        reg_ctrl     <= 32'h00000003;  // TX_EN + RX_EN by default
        reg_baud     <= CLK_FREQ / BAUD_RATE - 1;
        reg_irq_en   <= 3'b000;
        reg_irq_stat <= 3'b000;
    end else begin
        // Latch sticky error flags
        if (parity_err)  reg_irq_stat[2] <= 1'b1;
        if (frame_err)   reg_irq_stat[2] <= 1'b1;
        if (overrun_err) reg_irq_stat[2] <= 1'b1;
        if (tx_fifo_empty && !tx_busy) reg_irq_stat[0] <= 1'b1;
        if (rx_data_valid)             reg_irq_stat[1] <= 1'b1;

        if (apb_wr) begin
            case (PADDR[7:2])
                6'h00: reg_ctrl     <= PWDATA;
                6'h01: reg_baud     <= PWDATA[15:0];
                6'h06: reg_irq_en   <= PWDATA[2:0];
                6'h07: reg_irq_stat <= reg_irq_stat & ~PWDATA[2:0]; // W1C
                default: ;
            endcase
        end
    end
end

// ---------------------------------------------------------------------------
// APB read logic
// ---------------------------------------------------------------------------
wire [31:0] status_reg = {23'b0, overrun_err, frame_err, parity_err,
                          rx_fifo_empty, rx_fifo_full, rx_data_valid,
                          tx_fifo_empty,  tx_fifo_full, tx_busy};

always @(*) begin
    PRDATA = 32'h0;
    case (PADDR[7:2])
        6'h00: PRDATA = reg_ctrl;
        6'h01: PRDATA = {16'h0, reg_baud};
        6'h03: PRDATA = {24'h0, rx_rd_data};                          // RXDATA
        6'h04: PRDATA = status_reg;                                    // STATUS
        6'h05: PRDATA = {24'h0, rx_fifo_count, tx_fifo_count};        // FIFO_LVL
        6'h06: PRDATA = {29'h0, reg_irq_en};                          // IRQ_EN
        6'h07: PRDATA = {29'h0, reg_irq_stat};                        // IRQ_STAT
        default: PRDATA = 32'hDEADBEEF;
    endcase
end

// ---------------------------------------------------------------------------
// Interrupt generation
// ---------------------------------------------------------------------------
assign irq = |(reg_irq_en & reg_irq_stat);

endmodule