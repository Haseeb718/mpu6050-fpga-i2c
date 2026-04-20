// ============================================================================
// MPU6050 I2C Interface - Top Level Module
// ============================================================================
//
// Based on ADS1115 interface pattern
// Reads gyroscope values (X, Y, Z) from MPU6050
//
// MPU6050 I2C Address: 0x68 (AD0 pin tied to GND)
//
// IMPORTANT REGISTERS:
//   PWR_MGMT_1  (0x6B) - Write 0x00 to wake up from sleep
//   GYRO_XOUT_H (0x43) - X gyro high byte
//   GYRO_XOUT_L (0x44) - X gyro low byte
//   GYRO_YOUT_H (0x45) - Y gyro high byte
//   GYRO_YOUT_L (0x46) - Y gyro low byte
//   GYRO_ZOUT_H (0x47) - Z gyro high byte
//   GYRO_ZOUT_L (0x48) - Z gyro low byte
//
// CONFIGURATION:
//   FS_SEL=0 (+/- 250 deg/s) - default after reset
//
// ============================================================================

module top (
    input  wire       clk,           // 50 MHz system clock

    // I2C interface
    inout  wire       scl,           // I2C clock
    inout  wire       sda,           // I2C data


    // Gyroscope outputs (signed 16-bit values)
    output reg  signed [15:0] gyro_x,
    output reg  signed [15:0] gyro_y,
    output reg  signed [15:0] gyro_z,
    output reg                gyro_valid,

    // Debug outputs for ILA
    output wire       dbg_scl_in,
    output wire       dbg_sda_in,
    output wire       dbg_scl_oe,
    output wire       dbg_sda_oe,
    output wire [4:0] dbg_state,
    output wire       i2c_ack
);

    // ========================================================================
    // TIMING PARAMETERS
    // ========================================================================

    localparam CLOCKS_PER_INTERVAL = 25'd5000000;   // 100ms at 50MHz

    // ========================================================================
    // MPU6050 I2C PARAMETERS
    // ========================================================================

    localparam MPU6050_ADDR_W = 8'hD0;  // 0x68 << 1 | 0 (write)
    localparam MPU6050_ADDR_R = 8'hD1;  // 0x68 << 1 | 1 (read)

    // Register addresses
    localparam REG_PWR_MGMT_1  = 8'h6B;
    localparam REG_GYRO_XOUT_H = 8'h43;

    // ========================================================================
    // I2C COMMAND DEFINITIONS
    // ========================================================================

    localparam CMD_NONE  = 3'd0;
    localparam CMD_START = 3'd1;
    localparam CMD_STOP  = 3'd2;
    localparam CMD_WRITE = 3'd3;
    localparam CMD_READ  = 3'd4;

    // ========================================================================
    // INTERNAL SIGNALS
    // ========================================================================

    reg  [2:0] i2c_cmd      = CMD_NONE;
    reg  [7:0] i2c_data_out = 8'd0;
    reg        i2c_ack_send = 1'b1;
    reg        i2c_start    = 1'b0;
    wire [7:0] i2c_data_in;
    wire       i2c_busy;

    // ========================================================================
    // STATE DEFINITIONS
    // ========================================================================

    localparam ST_IDLE          = 5'd0;
    localparam ST_WAKE_START    = 5'd1;
    localparam ST_WAKE_ADDR     = 5'd2;
    localparam ST_WAKE_REG      = 5'd3;
    localparam ST_WAKE_DATA     = 5'd4;
    localparam ST_WAKE_STOP     = 5'd5;
    localparam ST_PTR_START     = 5'd6;
    localparam ST_PTR_ADDR      = 5'd7;
    localparam ST_PTR_REG       = 5'd8;
    localparam ST_PTR_STOP      = 5'd9;
    localparam ST_WAIT_INTERVAL = 5'd10;
    localparam ST_RD_START      = 5'd11;
    localparam ST_RD_ADDR       = 5'd12;
    localparam ST_RD_MSB        = 5'd13;
    localparam ST_RD_LSB        = 5'd14;
    localparam ST_RD_NEXT       = 5'd15;
    localparam ST_RD_STOP       = 5'd16;
    localparam ST_DONE          = 5'd17;
    localparam ST_ERROR         = 5'd18;

    reg [4:0]  state            = ST_IDLE;
    reg        seen_busy        = 1'b0;
    reg        error_flag       = 1'b0;
    reg [24:0] interval_counter = 25'd0;
    reg        interval_tick    = 1'b0;
    reg [2:0]  byte_counter     = 3'd0;

    // Temp storage for gyro readings
    reg [7:0] gyro_x_h, gyro_x_l;
    reg [7:0] gyro_y_h, gyro_y_l;
    reg [7:0] gyro_z_h, gyro_z_l;

    // ========================================================================
    // I2C MASTER INSTANCE
    // ========================================================================

    mpu6050 u_i2c (
        .clk(clk),
        .scl(scl),
        .sda(sda),
        .cmd_i(i2c_cmd),
        .data_i(i2c_data_out),
        .ack_i(i2c_ack_send),
        .start_i(i2c_start),
        .data_o(i2c_data_in),
        .ack_o(i2c_ack),
        .busy_o(i2c_busy),
        .dbg_scl_in(dbg_scl_in),
        .dbg_sda_in(dbg_sda_in),
        .dbg_scl_oe(dbg_scl_oe),
        .dbg_sda_oe(dbg_sda_oe)
    );

    // Debug assignments
    assign dbg_state = state;

    // ========================================================================
    // INTERVAL TIMER (100ms)
    // ========================================================================

    always @(posedge clk) begin
        interval_tick <= 1'b0;

        if (interval_counter == CLOCKS_PER_INTERVAL - 1) begin
            interval_counter <= 25'd0;
            interval_tick    <= 1'b1;
        end else begin
            interval_counter <= interval_counter + 1'b1;
        end
    end

    // ========================================================================
    // MAIN STATE MACHINE
    // ========================================================================

    always @(posedge clk) begin
        i2c_start  <= 1'b0;
        gyro_valid <= 1'b0;

        if (i2c_busy) begin
            seen_busy <= 1'b1;
        end

        case (state)

            // ================================================================
            // IDLE: Wait a bit then start initialization
            // ================================================================
            ST_IDLE: begin
                if (interval_tick) begin
                    state <= ST_WAKE_START;
                end
            end

            // ================================================================
            // WAKE UP: Write 0x00 to PWR_MGMT_1 (0x6B) to exit sleep mode
            // Sequence: START -> 0xD0 -> 0x6B -> 0x00 -> STOP
            // ================================================================
            ST_WAKE_START: begin
                i2c_cmd   <= CMD_START;
                i2c_start <= 1'b1;
                seen_busy <= 1'b0;
                state     <= ST_WAKE_ADDR;
            end

            ST_WAKE_ADDR: begin
                if (!i2c_busy && seen_busy) begin
                    i2c_cmd      <= CMD_WRITE;
                    i2c_data_out <= MPU6050_ADDR_W;
                    i2c_start    <= 1'b1;
                    seen_busy    <= 1'b0;
                    state        <= ST_WAKE_REG;
                end
            end

            ST_WAKE_REG: begin
                if (!i2c_busy && seen_busy) begin
                    if (i2c_ack) begin
                        error_flag <= 1'b1;
                        i2c_cmd    <= CMD_STOP;
                        i2c_start  <= 1'b1;
                        state      <= ST_ERROR;
                    end else begin
                        i2c_cmd      <= CMD_WRITE;
                        i2c_data_out <= REG_PWR_MGMT_1;
                        i2c_start    <= 1'b1;
                        seen_busy    <= 1'b0;
                        state        <= ST_WAKE_DATA;
                    end
                end
            end

            ST_WAKE_DATA: begin
                if (!i2c_busy && seen_busy) begin
                    if (i2c_ack) begin
                        error_flag <= 1'b1;
                        i2c_cmd    <= CMD_STOP;
                        i2c_start  <= 1'b1;
                        state      <= ST_ERROR;
                    end else begin
                        i2c_cmd      <= CMD_WRITE;
                        i2c_data_out <= 8'h00;  // Wake up: clear sleep bit
                        i2c_start    <= 1'b1;
                        seen_busy    <= 1'b0;
                        state        <= ST_WAKE_STOP;
                    end
                end
            end

            ST_WAKE_STOP: begin
                if (!i2c_busy && seen_busy) begin
                    if (i2c_ack) begin
                        error_flag <= 1'b1;
                        i2c_cmd    <= CMD_STOP;
                        i2c_start  <= 1'b1;
                        state      <= ST_ERROR;
                    end else begin
                        i2c_cmd   <= CMD_STOP;
                        i2c_start <= 1'b1;
                        seen_busy <= 1'b0;
                        state     <= ST_WAIT_INTERVAL;
                    end
                end
            end

            // ================================================================
            // WAIT: Give time for device to settle
            // ================================================================
            ST_WAIT_INTERVAL: begin
                if (!i2c_busy && seen_busy) begin
                    seen_busy <= 1'b0;
                end

                if (interval_tick && !seen_busy) begin
                    byte_counter <= 3'd0;
                    state        <= ST_PTR_START;
                end
            end

            // ================================================================
            // SET POINTER: Point to GYRO_XOUT_H register
            // ================================================================
            ST_PTR_START: begin
                i2c_cmd   <= CMD_START;
                i2c_start <= 1'b1;
                seen_busy <= 1'b0;
                state     <= ST_PTR_ADDR;
            end

            ST_PTR_ADDR: begin
                if (!i2c_busy && seen_busy) begin
                    i2c_cmd      <= CMD_WRITE;
                    i2c_data_out <= MPU6050_ADDR_W;
                    i2c_start    <= 1'b1;
                    seen_busy    <= 1'b0;
                    state        <= ST_PTR_REG;
                end
            end

            ST_PTR_REG: begin
                if (!i2c_busy && seen_busy) begin
                    if (i2c_ack) begin
                        error_flag <= 1'b1;
                        i2c_cmd    <= CMD_STOP;
                        i2c_start  <= 1'b1;
                        state      <= ST_ERROR;
                    end else begin
                        i2c_cmd      <= CMD_WRITE;
                        i2c_data_out <= REG_GYRO_XOUT_H;
                        i2c_start    <= 1'b1;
                        seen_busy    <= 1'b0;
                        state        <= ST_PTR_STOP;
                    end
                end
            end

            ST_PTR_STOP: begin
                if (!i2c_busy && seen_busy) begin
                    if (i2c_ack) begin
                        error_flag <= 1'b1;
                        i2c_cmd    <= CMD_STOP;
                        i2c_start  <= 1'b1;
                        state      <= ST_ERROR;
                    end else begin
                        i2c_cmd   <= CMD_STOP;
                        i2c_start <= 1'b1;
                        seen_busy <= 1'b0;
                        state     <= ST_RD_START;
                    end
                end
            end

            // ================================================================
            // READ: Repeated START -> 0xD1 -> Read 6 bytes
            // Uses auto-increment: reading from 0x43 gets 0x43,0x44,0x45,0x46,0x47,0x48
            // ================================================================
            ST_RD_START: begin
                if (!i2c_busy && seen_busy) begin
                    i2c_cmd   <= CMD_START;
                    i2c_start <= 1'b1;
                    seen_busy <= 1'b0;
                    state     <= ST_RD_ADDR;
                end
            end

            ST_RD_ADDR: begin
                if (!i2c_busy && seen_busy) begin
                    i2c_cmd      <= CMD_WRITE;
                    i2c_data_out <= MPU6050_ADDR_R;
                    i2c_start    <= 1'b1;
                    seen_busy    <= 1'b0;
                    state        <= ST_RD_MSB;
                end
            end

            ST_RD_MSB: begin
                if (!i2c_busy && seen_busy) begin
                    if (i2c_ack) begin
                        error_flag <= 1'b1;
                        i2c_cmd    <= CMD_STOP;
                        i2c_start  <= 1'b1;
                        state      <= ST_ERROR;
                    end else begin
                        // Read MSB, send ACK to continue
                        i2c_cmd      <= CMD_READ;
                        i2c_ack_send <= 1'b0;   // ACK (more bytes coming)
                        i2c_start    <= 1'b1;
                        seen_busy    <= 1'b0;
                        state        <= ST_RD_LSB;
                    end
                end
            end

            ST_RD_LSB: begin
                if (!i2c_busy && seen_busy) begin
                    // Store MSB based on current register
                    case (byte_counter)
                        3'd0: gyro_x_h <= i2c_data_in;
                        3'd1: gyro_y_h <= i2c_data_in;
                        3'd2: gyro_z_h <= i2c_data_in;
                        default: ;
                    endcase

                    // Read LSB
                    i2c_cmd      <= CMD_READ;
                    i2c_ack_send <= (byte_counter == 3'd2) ? 1'b1 : 1'b0;  // NACK on last byte
                    i2c_start    <= 1'b1;
                    seen_busy    <= 1'b0;
                    state        <= ST_RD_NEXT;
                end
            end

            ST_RD_NEXT: begin
                if (!i2c_busy && seen_busy) begin
                    // Store LSB
                    case (byte_counter)
                        3'd0: gyro_x_l <= i2c_data_in;
                        3'd1: gyro_y_l <= i2c_data_in;
                        3'd2: gyro_z_l <= i2c_data_in;
                        default: ;
                    endcase

                    if (byte_counter == 3'd2) begin
                        i2c_cmd   <= CMD_STOP;
                        i2c_start <= 1'b1;
                        seen_busy <= 1'b0;
                        state     <= ST_RD_STOP;
                    end else begin
                        byte_counter <= byte_counter + 1'b1;
                        seen_busy    <= 1'b1;
                        state        <= ST_RD_MSB;
                    end
                end
            end

            ST_RD_STOP: begin
                if (!i2c_busy && seen_busy) begin
                    // Combine bytes into signed 16-bit values
                    gyro_x     <= {gyro_x_h, gyro_x_l};
                    gyro_y     <= {gyro_y_h, gyro_y_l};
                    gyro_z     <= {gyro_z_h, gyro_z_l};
                    gyro_valid <= 1'b1;
                    seen_busy  <= 1'b0;
                    state      <= ST_DONE;
                end
            end

            // ================================================================
            // DONE: Wait for next interval then read again
            // ================================================================
            ST_DONE: begin
                if (interval_tick) begin
                    byte_counter <= 3'd0;
                    state        <= ST_PTR_START;
                end
            end

            // ================================================================
            // ERROR: Try to recover
            // ================================================================
            ST_ERROR: begin
                if (!i2c_busy && seen_busy) begin
                    seen_busy <= 1'b0;
                end

                if (!seen_busy) begin
                    if (interval_tick) begin
                        error_flag <= 1'b0;
                        state      <= ST_IDLE;
                    end
                end
            end

            default: begin
                state <= ST_IDLE;
            end
        endcase
    end

endmodule
