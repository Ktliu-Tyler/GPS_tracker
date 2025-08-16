# NMEA GPS to CAN Direct Converter

This module provides a direct conversion from NMEA GPS data to CAN bus messages without using intermediate ROS topics. It combines the functionality of the original NMEA driver, parser, and CAN publisher into a single efficient node.

## Features

- **Direct NMEA to CAN conversion**: Bypasses ROS topics for better performance
- **NMEA parsing**: Supports GGA, RMC, VTG, GST, and HDT sentences
- **CAN message encoding**: Uses Ecumaster-compatible format
- **Position validation**: Filters GPS positions based on expected ranges
- **Timestamp broadcasting**: Sends periodic timestamps to CAN bus
- **Raw NMEA forwarding**: Forwards complete NMEA strings to CAN for logging

## CAN Message IDs

| ID Range | Description |
|----------|-------------|
| 0x100    | Timestamp (milliseconds since midnight + days since 1984) |
| 0x400    | GPS basic position (latitude + longitude) |
| 0x401    | GPS altitude |
| 0x402    | GPS status (fix type, satellites, HDOP, valid flag) |
| 0x403-0x406 | Velocity data (VX, VY, VZ, speed magnitude) |
| 0x500    | NMEA string header (length + frame count) |
| 0x501+   | NMEA string data frames (8 bytes per frame) |

## Usage

### Direct execution
```bash
ros2 run gps_tracker nmea_gps_can
```

### Using launch file
```bash
ros2 launch gps_tracker launch_nmea_gps_can.py
```

### With custom parameters
```bash
ros2 launch gps_tracker launch_nmea_gps_can.py \
    serial_port:=/dev/ttyUSB0 \
    serial_baudrate:=115200 \
    can_channel:=can0 \
    longitude_range:=121.0 \
    latitude_range:=25.0
```

## Parameters

- `serial_port` (default: `/dev/ttyUSB0`): Serial port for GPS receiver
- `serial_baudrate` (default: `115200`): Serial communication baudrate
- `can_channel` (default: `can0`): CAN interface name
- `longitude_range` (default: `121.0`): Expected longitude for position validation
- `latitude_range` (default: `25.0`): Expected latitude for position validation
- `frame_id` (default: `gps`): Frame ID for GPS data

## Dependencies

- `rclpy`: ROS 2 Python client library
- `python-can`: Python CAN bus library
- `pyserial`: Python serial communication library

## NMEA Sentence Support

### GGA (Global Positioning System Fix Data)
- Position (latitude, longitude, altitude)
- Fix quality and number of satellites
- Horizontal dilution of precision (HDOP)

### RMC (Recommended Minimum)
- Position, speed, and course
- Fix validity status

### VTG (Track Made Good and Ground Speed)
- Course and speed information

### GST (GPS Pseudorange Noise Statistics)
- Error estimation (parsed but not currently used)

### HDT (Heading - True)
- True heading information

## Position Validation

The converter includes position validation to filter out erroneous GPS readings:

1. **Range check**: Positions must be within 1 degree of expected coordinates
2. **Continuity check**: Large position jumps are filtered using previous valid position
3. **Fix validation**: Only processes data when GPS has a valid fix

## Error Handling

- **Serial errors**: Automatic reconnection attempts
- **CAN errors**: Logged with error details
- **NMEA parsing errors**: Invalid sentences are logged and skipped
- **Checksum validation**: All NMEA sentences are validated before processing

## Example Output

```
[INFO] [nmea_gps_can_converter]: NMEA GPS to CAN Converter started successfully
[INFO] [nmea_gps_can_converter]: Serial: /dev/ttyUSB0 @ 115200
[INFO] [nmea_gps_can_converter]: CAN: can0
[INFO] [nmea_gps_can_converter]: Publishing GPS data directly to CAN bus
[INFO] [nmea_gps_can_converter]: GPS CAN sent: Lat=25.123456, Lon=121.654321, Alt=123.4m, Fix=1, Sats=8
[INFO] [nmea_gps_can_converter]: Velocity CAN sent: Speed=45.67km/h, Course=123.4°, VX=10.123m/s, VY=8.456m/s
```

## Building

Make sure to build the package after making changes:

```bash
cd /path/to/workspace
colcon build --packages-select gps_tracker
source install/setup.bash
```

## CAN Interface Setup

Before running, ensure your CAN interface is properly configured:

```bash
# Bring up CAN interface
sudo ip link set can0 up type can bitrate 500000

# Verify CAN interface
ip link show can0
```
