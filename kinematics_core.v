// ==========================================
// MODULE: kinematics_core
// DESCRIPTION: Hardware accelerator for Differential Drive Kinematics.
// Computes Inverse Kinematics, Forward Kinematics, CORDIC (Sine/Cosine), 
// and Euler Integration using Q16.16 Fixed-Point Arithmetic.
//
// HOW TO ADAPT THIS TO YOUR CUSTOM ROBOT:
// All parameters must be converted to Q16.16 Fixed-Point integers.
// Formula: Q16.16_Value = Integer( Real_Value * 65536 )
//
// Example for a robot with R = 0.05m, L = 0.15m, dt = 0.05s:
// CONST_R_INV    -> (1 / 0.05) * 65536 = 20 * 65536 = 1310720
// CONST_L_OVER_R -> (0.15 / 0.05) * 65536 = 3 * 65536 = 196608
// CONST_R_OVER_2 -> (0.05 / 2) * 65536 = 0.025 * 65536 = 1638
// CONST_DT       -> (0.05) * 65536 = 3276
// ==========================================

module kinematics_core #(
    parameter signed [31:0] CONST_R_INV    = 32'd1310720,
    parameter signed [31:0] CONST_L_OVER_R = 32'd196608,
    parameter signed [31:0] CONST_R_OVER_2 = 32'd1638,
    parameter signed [31:0] CONST_DT       = 32'd3276
)(
    input clk,
    input reset,
    input start,                    // Trigger to start calculation

    // Inputs from PC (Target velocities)
    input signed [31:0] v_target,
    input signed [31:0] omega_target,

    // Outputs to Motors (Inverse Kinematics results)
    output reg signed [31:0] phi_r_cmd,
    output reg signed [31:0] phi_l_cmd,

    // Outputs for Odometry (Robot's actual position in space)
    output reg signed [31:0] x_pos,
    output reg signed [31:0] y_pos,
    output reg signed [31:0] theta_pos,

    // Benchmarking
    output reg [31:0] cycle_count,  // Hardware execution time tracker
    output reg done                 // Pulses high when calculation is finished
);

    // CORDIC Base Gain (approx 0.607252 * 65536). Fixed mathematical constant.
    localparam signed [31:0] CORDIC_GAIN = 32'd39797;

    // --- 64-BIT OVERFLOW PROTECTION WIRES ---
    // Multiplication in Q16.16 generates Q32.32, which needs 64 bits to prevent clipping
    wire signed [63:0] mult_v_r     = $signed(v_target) * $signed(CONST_R_INV);
    wire signed [63:0] mult_w_l     = $signed(omega_target) * $signed(CONST_L_OVER_R);
    wire signed [63:0] mult_fwd     = $signed(phi_r_cmd + phi_l_cmd) * $signed(CONST_R_OVER_2);

    // Calculate new orientation immediately to prevent pipeline delays
    wire signed [63:0] mult_theta   = $signed(omega_target) * $signed(CONST_DT);

    wire signed [63:0] mult_x_vel   = $signed(v_actual) * $signed(cordic_x);
    wire signed [63:0] mult_x_pos   = $signed(mult_x_vel >>> 16) * $signed(CONST_DT);

    wire signed [63:0] mult_y_vel   = $signed(v_actual) * $signed(cordic_y);
    wire signed [63:0] mult_y_pos   = $signed(mult_y_vel >>> 16) * $signed(CONST_DT);


    // CORDIC ATAN Table (Q16.16 Radians)
    wire signed [31:0] atan_table [0:15];
    assign atan_table[0]  = 32'd51472; assign atan_table[1]  = 32'd30386;
    assign atan_table[2]  = 32'd16055; assign atan_table[3]  = 32'd8150;
    assign atan_table[4]  = 32'd4091;  assign atan_table[5]  = 32'd2047;
    assign atan_table[6]  = 32'd1024;  assign atan_table[7]  = 32'd512;
    assign atan_table[8]  = 32'd256;   assign atan_table[9]  = 32'd128;
    assign atan_table[10] = 32'd64;    assign atan_table[11] = 32'd32;
    assign atan_table[12] = 32'd16;    assign atan_table[13] = 32'd8;
    assign atan_table[14] = 32'd4;     assign atan_table[15] = 32'd2;

    // FSM States
    localparam STATE_IDLE        = 3'd0;
    localparam STATE_INV_KIN     = 3'd1;
    localparam STATE_FWD_KIN     = 3'd2;
    localparam STATE_CORDIC_INIT = 3'd3;
    localparam STATE_CORDIC_LOOP = 3'd4;
    localparam STATE_INTEGRATE   = 3'd5;
    localparam STATE_DONE        = 3'd6;

    reg [2:0] state;
    reg signed [31:0] v_actual;
    reg signed [31:0] cordic_x, cordic_y, cordic_z;
    reg [4:0] cordic_step;

    initial begin
        state = STATE_IDLE;
        x_pos = 0; y_pos = 0; theta_pos = 0;
        done = 0; cycle_count = 0;
    end

    always @(posedge clk) begin
        if (reset) begin // Active-High logic based on physical constraint modifications
            state <= STATE_IDLE;
            x_pos <= 0; y_pos <= 0; theta_pos <= 0;
            done <= 0;
            cycle_count <= 0;
        end else begin
            case (state)

                STATE_IDLE: begin
                    done <= 0;
                    if (start) begin
                        cycle_count <= 0;
                        state <= STATE_INV_KIN;
                    end
                end

                // STEP 1: Inverse Kinematics (Target Velocity -> Wheel Commands)
                STATE_INV_KIN: begin
                    cycle_count <= cycle_count + 1;
                    // Apply '>>> 16' to shift back to Q16.16 format
                    phi_r_cmd <= (mult_v_r >>> 16) + (mult_w_l >>> 16);
                    phi_l_cmd <= (mult_v_r >>> 16) - (mult_w_l >>> 16);
                    state <= STATE_FWD_KIN;
                end

                // STEP 2: Forward Kinematics (Wheel Constraints -> Robot Actual Velocity)
                STATE_FWD_KIN: begin
                    cycle_count <= cycle_count + 1;
                    v_actual <= (mult_fwd >>> 16);
                    theta_pos <= theta_pos + (mult_theta >>> 16);
                    state <= STATE_CORDIC_INIT;
                end

                // STEP 3: Setup CORDIC
                STATE_CORDIC_INIT: begin
                    cycle_count <= cycle_count + 1;
                    cordic_x <= CORDIC_GAIN;
                    cordic_y <= 0;
                    cordic_z <= theta_pos; // Target angle
                    cordic_step <= 0;
                    state <= STATE_CORDIC_LOOP;
                end

                // STEP 4: CORDIC Loop (Computes Sine and Cosine without multipliers)
                STATE_CORDIC_LOOP: begin
                    cycle_count <= cycle_count + 1;
                    if (cordic_z >= 0) begin
                        cordic_x <= cordic_x - (cordic_y >>> cordic_step);
                        cordic_y <= cordic_y + (cordic_x >>> cordic_step);
                        cordic_z <= cordic_z - atan_table[cordic_step];
                    end else begin
                        cordic_x <= cordic_x + (cordic_y >>> cordic_step);
                        cordic_y <= cordic_y - (cordic_x >>> cordic_step);
                        cordic_z <= cordic_z + atan_table[cordic_step];
                    end

                    if (cordic_step == 15) begin
                        state <= STATE_INTEGRATE;
                    end else begin
                        cordic_step <= cordic_step + 1;
                    end
                end

                // STEP 5: Euler Integration (Update X and Y Positions)
                STATE_INTEGRATE: begin
                    cycle_count <= cycle_count + 1;
                    x_pos <= x_pos + (mult_x_pos >>> 16);
                    y_pos <= y_pos + (mult_y_pos >>> 16);
                    state <= STATE_DONE;
                end

                // STEP 6: Finished. Signal Top Module
                STATE_DONE: begin
                    done <= 1;
                    state <= STATE_IDLE;
                end

            endcase
        end
    end
endmodule
