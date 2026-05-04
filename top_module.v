// ==========================================
// MODULE: top_module
// DESCRIPTION: Main controller for the Differential Drive IP Core.
//
// UART DATA FRAME MAP (115200 Baud, Big Endian):
// ---------------------------------------------------------
// RX (PC to FPGA) - Total 8 Bytes:
// Byte 0-3 : Target Linear Velocity (V)
// Byte 4-7 : Target Angular Velocity (Omega)
//
// TX (FPGA to PC) - Total 24 Bytes:
// Byte 0-3  : Right Wheel Command (phi_r)
// Byte 4-7  : Left Wheel Command (phi_l)
// Byte 8-11 : Estimated X Position (x)
// Byte 12-15: Estimated Y Position (y)
// Byte 16-19: Estimated Orientation (theta)
// Byte 20-23: Hardware Cycle Count (Unsigned Int)
// ==========================================

module top_module(
    input clk,
    input reset,            // Physical button on FPGA
    input uart_rx,          // Physical Pin 70 (From PC)
    output uart_tx,         // Physical Pin 69 (To PC)
    output [1:0] leds       // Debug LEDs
);

    // ==========================================
    // INTERNAL WIRES & REGISTERS
    // ==========================================

    wire [7:0] rx_data;
    wire rx_done;

    reg tx_start;
    reg [7:0] tx_data;
    wire tx_busy;

    reg kin_start;
    wire kin_done;
    wire signed [31:0] kin_phi_r, kin_phi_l;
    wire signed [31:0] kin_x, kin_y, kin_theta;
    wire [31:0] kin_cycles;

    reg signed [31:0] target_v, target_omega;

    // Buffers for packing/unpacking payload bytes
    reg [7:0] rx_buffer [0:7];   // Receives 8 bytes (V, Omega)
    reg [7:0] tx_buffer [0:23];  // Sends 24 bytes (phi_r, phi_l, x, y, theta, cycles)

    reg [3:0] rx_index;
    reg [4:0] tx_index;

    // Main FSM States
    localparam STATE_RX      = 3'd0;
    localparam STATE_CALC    = 3'd1;
    localparam STATE_PACK    = 3'd2;
    localparam STATE_TX      = 3'd3;
    localparam STATE_TX_WAIT = 3'd4; // Prevents race conditions during UART TX

    reg [2:0] state;

    // Debug LEDs (Active-Low: 0=ON, 1=OFF)
    // LED 0 indicates RX mode. LED 1 indicates TX mode.
    assign leds[0] = (state == STATE_RX) ? 1'b0 : 1'b1;
    assign leds[1] = (state == STATE_TX || state == STATE_TX_WAIT) ? 1'b0 : 1'b1;

    // ==========================================
    // MODULE INSTANTIATIONS
    // ==========================================

    uart_rx receiver (
        .clk(clk),
        .rx_line(uart_rx),
        .data_out(rx_data),
        .rx_done(rx_done)
    );

    uart_tx transmitter (
        .clk(clk),
        .start_cmd(tx_start),
        .data_in(tx_data),
        .tx_line(uart_tx),
        .busy(tx_busy)
    );

    // The IP Core is instantiated here.
    // You can override parameters here for different robot sizes in the future.
    kinematics_core math_core (
        .clk(clk),
        .reset(reset),
        .start(kin_start),
        .v_target(target_v),
        .omega_target(target_omega),
        .phi_r_cmd(kin_phi_r),
        .phi_l_cmd(kin_phi_l),
        .x_pos(kin_x),
        .y_pos(kin_y),
        .theta_pos(kin_theta),
        .cycle_count(kin_cycles),
        .done(kin_done)
    );

    // ==========================================
    // MAIN STATE MACHINE
    // ==========================================

    initial begin
        state = STATE_RX;
        rx_index = 0;
        tx_index = 0;
        tx_start = 0;
        kin_start = 0;
    end

    always @(posedge clk) begin
        if (reset) begin
            state <= STATE_RX;
            rx_index <= 0;
            tx_index <= 0;
            tx_start <= 0;
            kin_start <= 0;
        end else begin
            case (state)

                // 1. RECEIVE DATA FROM PYTHON (Deserialization)
                STATE_RX: begin
                    if (rx_done) begin
                        rx_buffer[rx_index] <= rx_data;
                        if (rx_index == 7) begin
                            // Reconstruct 32-bit values from 8-bit slices (Big Endian)
                            target_v     <= {rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]};
                            target_omega <= {rx_buffer[4], rx_buffer[5], rx_buffer[6], rx_buffer[7]};

                            rx_index <= 0;
                            kin_start <= 1; // Trigger the calculation core
                            state <= STATE_CALC;
                        end else begin
                            rx_index <= rx_index + 1;
                        end
                    end
                end

                // 2. WAIT FOR THE MATH CORE
                STATE_CALC: begin
                    kin_start <= 0; // Drop trigger immediately (1-cycle pulse)
                    if (kin_done) begin
                        state <= STATE_PACK;
                    end
                end

                // 3. PACK RESULTS FOR TRANSMISSION (Serialization)
                STATE_PACK: begin
                    // Break down 32-bit results into 8-bit byte slices
                    {tx_buffer[0], tx_buffer[1], tx_buffer[2], tx_buffer[3]}     <= kin_phi_r;
                    {tx_buffer[4], tx_buffer[5], tx_buffer[6], tx_buffer[7]}     <= kin_phi_l;
                    {tx_buffer[8], tx_buffer[9], tx_buffer[10], tx_buffer[11]}   <= kin_x;
                    {tx_buffer[12], tx_buffer[13], tx_buffer[14], tx_buffer[15]} <= kin_y;
                    {tx_buffer[16], tx_buffer[17], tx_buffer[18], tx_buffer[19]} <= kin_theta;
                    {tx_buffer[20], tx_buffer[21], tx_buffer[22], tx_buffer[23]} <= kin_cycles;

                    tx_index <= 0;
                    state <= STATE_TX;
                end

                // 4. SEND DATA TO PYTHON (Trigger TX)
                STATE_TX: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data <= tx_buffer[tx_index];
                        tx_start <= 1;
                    end else if (tx_start && tx_busy) begin
                        tx_start <= 0;
                        state <= STATE_TX_WAIT;
                    end
                end

                // 5. WAIT FOR BYTE TRANSMISSION
                STATE_TX_WAIT: begin
                    if (!tx_busy) begin
                        if (tx_index == 23) begin
                            // All 24 bytes sent, return to receiving state
                            tx_index <= 0;
                            state <= STATE_RX;
                        end else begin
                            tx_index <= tx_index + 1;
                            state <= STATE_TX;
                        end
                    end
                end

            endcase
        end
    end

endmodule
