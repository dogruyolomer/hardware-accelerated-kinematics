// ==========================================
// MODULE: uart_tx
// DESCRIPTION: UART Transmitter module. Serializes 8-bit parallel
// data into serial bits at 115200 baud rate (27 MHz clock).
// CLOCKS_PER_BIT = 27,000,000 / 115200 = ~234
// ==========================================

module uart_tx (
    input clk,
    input start_cmd,
    input [7:0] data_in,
    output reg tx_line,
    output reg busy
);

    parameter CLOCKS_PER_BIT = 234;

    localparam STATE_IDLE  = 3'd0;
    localparam STATE_START = 3'd1;
    localparam STATE_DATA  = 3'd2;
    localparam STATE_STOP  = 3'd3;

    reg [2:0] state = STATE_IDLE;
    reg [15:0] clock_count = 0;
    reg [2:0] bit_index = 0;
    reg [7:0] data_buffer = 0;

    initial begin
        tx_line = 1'b1; // Line is high when idle
        busy = 0;
    end

    always @(posedge clk) begin
        case (state)
            STATE_IDLE: begin
                tx_line <= 1'b1;
                clock_count <= 0;
                bit_index <= 0;
                if (start_cmd == 1) begin
                    data_buffer <= data_in;
                    busy <= 1;
                    state <= STATE_START;
                end else begin
                    busy <= 0;
                end
            end

            STATE_START: begin
                tx_line <= 1'b0; // Send start bit
                if (clock_count == CLOCKS_PER_BIT - 1) begin
                    clock_count <= 0;
                    state <= STATE_DATA;
                end else begin
                    clock_count <= clock_count + 1;
                end
            end

            STATE_DATA: begin
                tx_line <= data_buffer[bit_index];
                if (clock_count == CLOCKS_PER_BIT - 1) begin
                    clock_count <= 0;
                    if (bit_index == 7) begin
                        state <= STATE_STOP;
                    end else begin
                        bit_index <= bit_index + 1;
                    end
                end else begin
                    clock_count <= clock_count + 1;
                end
            end

            STATE_STOP: begin
                tx_line <= 1'b1; // Send stop bit
                if (clock_count == CLOCKS_PER_BIT - 1) begin
                    clock_count <= 0;
                    state <= STATE_IDLE;
                end else begin
                    clock_count <= clock_count + 1;
                end
            end

            default: state <= STATE_IDLE;
        endcase
    end
endmodule
