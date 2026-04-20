# MPU6050 Gyroscope Reader — Zynq-7010 FPGA

A fully hardware-implemented I2C interface for the MPU6050 IMU sensor, running on a Zynq-7010 FPGA at 50 MHz. Gyroscope X, Y, Z values are read over I2C and exposed to the ARM PS via AXI-mapped registers.

\---

## Project Structure

```
mpu6050-zynq/
├── rtl/
│   ├── top.v          # MPU6050 state machine + AXI outputs
│   └── mpu6050.v      # I2C master controller
├── vitis/
│   └── main.c         # ARM PS application (reads gyro over AXI)
├── Makefile
└── README.md
```

\---

## Hardware

|Component|Detail|
|-|-|
|FPGA Board|Zynq-7010 (e.g. PYNQ-Z1/Z2)|
|Sensor|MPU6050 (I2C address 0x68)|
|I2C Speed|100 kHz (standard mode)|
|System Clock|50 MHz|
|Pull-up resistors|4.7kΩ on SCL and SDA to 3.3V|

### Pin Connections

|MPU6050 Pin|FPGA Pin|
|-|-|
|VCC|3.3V|
|GND|GND|
|SCL|FPGA SCL (with 4.7kΩ pull-up)|
|SDA|FPGA SDA (with 4.7kΩ pull-up)|
|AD0|GND (sets I2C address to 0x68)|

\---

## How It Works

1. On startup, the FPGA wakes the MPU6050 by writing `0x00` to `PWR\_MGMT\_1` (reg `0x6B`)
2. Every 100ms, it sets the register pointer to `GYRO\_XOUT\_H` (`0x43`)
3. It then performs a burst read of 6 bytes (X\_H, X\_L, Y\_H, Y\_L, Z\_H, Z\_L)
4. The 16-bit signed values are assembled and written to AXI registers
5. The ARM core reads those registers and prints deg/s values over UART

\---

\---

## Output (UART)

```
Axis X = 42    Axis Y = -17    Axis Z = 3
Axis X = 40    Axis Y = -18    Axis Z = 2
```

Raw LSB values. At FS\_SEL=0, divide by 131 for deg/s.

\---

## MPU6050 Register Map (Used)

|Register|Address|Description|
|-|-|-|
|PWR\_MGMT\_1|0x6B|Power management|
|GYRO\_XOUT\_H|0x43|X-axis gyro high byte|
|GYRO\_XOUT\_L|0x44|X-axis gyro low byte|
|GYRO\_YOUT\_H|0x45|Y-axis gyro high byte|
|GYRO\_YOUT\_L|0x46|Y-axis gyro low byte|
|GYRO\_ZOUT\_H|0x47|Z-axis gyro high byte|
|GYRO\_ZOUT\_L|0x48|Z-axis gyro low byte|

\---

## License

MIT

