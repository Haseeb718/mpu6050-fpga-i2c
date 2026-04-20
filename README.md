# MPU6050 Gyroscope Reader — FPGA I2C Interface

A fully hardware-implemented I2C interface for the MPU6050 IMU sensor. Reads gyroscope X, Y, Z values over I2C and exposes them as 16-bit signed outputs. Designed for FPGAs with a 50 MHz system clock, with AXI readout demonstrated on Xilinx devices.

---

## Project Structure

```
mpu6050-fpga-i2c/
├── rtl/
│   ├── top.v          # MPU6050 state machine + gyro outputs
│   └── mpu6050.v      # I2C master controller
├── vitis/
│   └── main.c         # ARM PS application (reads gyro over AXI)
├── Makefile
└── README.md
```

---

## Hardware

| Component         | Detail                        |
|------------------|-------------------------------|
| FPGA              | Any FPGA with 50 MHz clock    |
| Sensor            | MPU6050 (I2C address 0x68)    |
| I2C Speed         | 100 kHz (standard mode)       |
| System Clock      | 50 MHz                        |
| Pull-up resistors | 4.7kΩ on SCL and SDA to 3.3V |

> **Note:** The open-drain I/O uses Xilinx `IOBUF` primitives in `mpu6050.v`. If targeting a non-Xilinx FPGA, replace the `IOBUF` instances with the equivalent tristate primitive for your device (e.g. `ALT_IOBUF` for Intel/Altera).

### Pin Connections

| MPU6050 Pin | FPGA Pin                       |
|-------------|--------------------------------|
| VCC         | 3.3V                           |
| GND         | GND                            |
| SCL         | FPGA I/O (with 4.7kΩ pull-up) |
| SDA         | FPGA I/O (with 4.7kΩ pull-up) |
| AD0         | GND (sets I2C address to 0x68) |

---

## How It Works

1. On startup, the FPGA wakes the MPU6050 by writing `0x00` to `PWR_MGMT_1` (reg `0x6B`)
2. Every 100ms, it sets the register pointer to `GYRO_XOUT_H` (`0x43`)
3. It then performs a burst read of 6 bytes (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
4. The 16-bit signed values are assembled into `gyro_x`, `gyro_y`, `gyro_z` output registers
5. On Xilinx devices, the ARM core reads those registers over AXI and prints values over UART

---

## Building

```bash
# Synthesize and implement (Vivado)
make vivado

# Build Vitis software project (Xilinx only)
make vitis

# Clean all outputs
make clean
```

For non-Xilinx flows, add the RTL files to your toolchain (Quartus, Yosys, etc.) and synthesize `top.v` as the top module.

---

## Output (UART — Xilinx example)

```
Axis X = 42    Axis Y = -17    Axis Z = 3
Axis X = 40    Axis Y = -18    Axis Z = 2
```

Raw LSB values. At FS_SEL=0, divide by 131 for deg/s.

---

## MPU6050 Register Map (Used)

| Register     | Address | Description            |
|--------------|---------|------------------------|
| PWR_MGMT_1   | 0x6B    | Power management       |
| GYRO_XOUT_H  | 0x43    | X-axis gyro high byte  |
| GYRO_XOUT_L  | 0x44    | X-axis gyro low byte   |
| GYRO_YOUT_H  | 0x45    | Y-axis gyro high byte  |
| GYRO_YOUT_L  | 0x46    | Y-axis gyro low byte   |
| GYRO_ZOUT_H  | 0x47    | Z-axis gyro high byte  |
| GYRO_ZOUT_L  | 0x48    | Z-axis gyro low byte   |

---

## License

MIT
