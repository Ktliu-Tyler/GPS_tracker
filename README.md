# GPS Tracker: ROS 2 and CAN Experiments

A collection of GPS acquisition, logging, visualization, and CAN communication experiments organized as a ROS 2 Python package. It records the development of several ways to move receiver data into a vehicle telemetry workflow and inspect the resulting position information.

## What the project explores

- Reading GPS/NMEA data from a serial receiver.
- Publishing and consuming GPS-related ROS 2 messages.
- Recording GPS information to CSV and raw NMEA logs.
- Visualizing routes and positions through plotting and map-oriented scripts.
- Encoding GPS data into CAN frames and decoding received frames.
- Working with RTK/NTRIP and u-blox receiver-related experiments.
- Converting NMEA directly to CAN within a ROS 2 node, bypassing intermediate topics.

## Structure and technologies

The package uses `rclpy` and standard ROS message packages, with `pyserial` and `python-can` for hardware communication. Individual scripts add plotting or receiver-specific capabilities. Multiple similarly named files preserve different implementation experiments rather than one uniform interface.

| Path | Description |
| --- | --- |
| [gps_tracker](gps_tracker) | GPS drivers, publishers, receivers, loggers, and visualizers |
| [gps_tracker/new](gps_tracker/new) | Integrated NMEA-to-CAN node and its documentation |
| [gps_tracker/launch](gps_tracker/launch) | ROS 2 launch configurations |
| [config/ublox_config_template.yaml](config/ublox_config_template.yaml) | Receiver configuration template |
| [docs/UBLOX_ZED_F9X_README.md](docs/UBLOX_ZED_F9X_README.md) | u-blox ZED-F9x notes |
| [setup.py](setup.py), [package.xml](package.xml) | Package metadata and registered executable entry points |

## Personal development record

This repository captures the process of integrating GPS data with a robotics/vehicle software stack. The different publisher and decoder versions are useful for tracing changes in message layout, update rate, and data flow; they should not be mixed without checking the corresponding encoding.

For the direct converter's design, see [its module README](gps_tracker/new/README.md). A separate, non-ROS sender is maintained in [GPS_nturt](https://github.com/Ktliu-Tyler/GPS_nturt).
