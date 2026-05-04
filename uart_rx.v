// ==========================================
// MODULE: uart_rx
// DESCRIPTION: UART Receiver module. Deserializes incoming serial
// bits into 8-bit parallel data at 115200 baud rate (27 MHz clock).
// CLOCKS_PER_BIT = 27,000,000 / 115200 = ~234
// ==========================================

module uart_rx (
    input clk,
    input rx_line,
    output reg [7:0] data_out,
    output reg rx_done
);

    parameter CLOCKS_PER_BIT = 234;

    localparam STATE_IDLE  = 3'd0;
    localparam STATE_START = 3'd1;
    localparam STATE_DATA  = 3'd2;
    localparam STATE_STOP  = 3'd3;
    localparam STATE_CLEAN = 3'd4;

    reg [2:0] state = STATE_IDLE;
    reg [15:0] clock_count = 0;
    reg [2:0] bit_index = 0;

    always @(posedge clk) begin
        case (state)
            STATE_IDLE: begin
                rx_done <= 0;
                clock_count <= 0;
                bit_index <= 0;
                if (rx_line == 0) begin // Start bit detected
                    state <= STATE_START;
                end
            end

            STATE_START: begin
                if (clock_count == (CLOCKS_PER_BIT / 2)) begin
                    if (rx_line == 0) begin // Verify start bit
                        clock_count <= 0;
                        state <= STATE_DATA;
                    end else begin
                        state <= STATE_IDLE; // False alarm
                    end
                end else begin
                    clock_count <= clock_count + 1;
                end
            end

            STATE_DATA: begin
                if (clock_count == CLOCKS_PER_BIT - 1) begin
                    clock_count <= 0;
                    data_out[bit_index] <= rx_line;
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
                if (clock_count == CLOCKS_PER_BIT - 1) begin
                    rx_done <= 1;
                    clock_count <= 0;
                    state <= STATE_CLEAN;
                end else begin
                    clock_count <= clock_count + 1;
                end
            end

            STATE_CLEAN: begin
                rx_done <= 0;
                state <= STATE_IDLE;
            end

            default: state <= STATE_IDLE;
        endcase
    end
endmodule
