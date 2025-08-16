#!/usr/bin/env python3
"""
NMEA GPS to CAN Direct Converter (Pure Python Version)
Converts NMEA GPS data directly to CAN messages without ROS2.
Combines NMEA parsing, GPS processing, and CAN transmission in one standalone application.
"""

import can
import struct
import time
import math
import calendar
import serial
import threading
import logging
import signal
import sys
from datetime import datetime
import argparse

# NMEA Parser Functions
def safe_float(field):
    """Safely convert field to float, return NaN if conversion fails"""
    try:
        return float(field)
    except (ValueError, TypeError):
        return float('NaN')

def safe_int(field):
    """Safely convert field to int, return 0 if conversion fails"""
    try:
        return int(field)
    except (ValueError, TypeError):
        return 0

def convert_latitude(field):
    """Convert NMEA latitude format (DDMM.MMMM) to decimal degrees"""
    if not field or len(field) < 4:
        return float('NaN')
    return safe_float(field[0:2]) + safe_float(field[2:]) / 60.0

def convert_longitude(field):
    """Convert NMEA longitude format (DDDMM.MMMM) to decimal degrees"""
    if not field or len(field) < 5:
        return float('NaN')
    return safe_float(field[0:3]) + safe_float(field[3:]) / 60.0

def convert_time(nmea_utc):
    """Convert NMEA UTC time to Unix timestamp"""
    if not nmea_utc or len(nmea_utc) < 6:
        return float('NaN')
    
    # Get current time in UTC for date information
    utc_struct = time.gmtime()
    utc_list = list(utc_struct)
    
    # If any time field is empty, return NaN
    if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6]:
        return float('NaN')
    
    try:
        hours = int(nmea_utc[0:2])
        minutes = int(nmea_utc[2:4])
        seconds = int(nmea_utc[4:6])
        utc_list[3] = hours
        utc_list[4] = minutes
        utc_list[5] = seconds
        unix_time = calendar.timegm(tuple(utc_list))
        return unix_time
    except ValueError:
        return float('NaN')

def convert_status_flag(status_flag):
    """Convert NMEA status flag to boolean"""
    return status_flag == "A"

def convert_knots_to_mps(knots):
    """Convert knots to meters per second"""
    return safe_float(knots) * 0.514444444444

def convert_deg_to_rads(degs):
    """Convert degrees to radians"""
    return math.radians(safe_float(degs))

def check_nmea_checksum(nmea_sentence):
    """Verify NMEA sentence checksum"""
    if '*' not in nmea_sentence:
        return False
    
    try:
        sentence, checksum = nmea_sentence.split('*')
        sentence = sentence.lstrip('$')
        
        calculated_checksum = 0
        for char in sentence:
            calculated_checksum ^= ord(char)
        
        return format(calculated_checksum, '02X') == checksum.upper()
    except (ValueError, IndexError):
        return False

# NMEA parsing configuration
PARSE_MAPS = {
    "GGA": [
        ("fix_type", int, 6),
        ("latitude", convert_latitude, 2),
        ("latitude_direction", str, 3),
        ("longitude", convert_longitude, 4),
        ("longitude_direction", str, 5),
        ("altitude", safe_float, 9),
        ("mean_sea_level", safe_float, 11),
        ("hdop", safe_float, 8),
        ("num_satellites", safe_int, 7),
        ("utc_time", convert_time, 1),
    ],
    "RMC": [
        ("utc_time", convert_time, 1),
        ("fix_valid", convert_status_flag, 2),
        ("latitude", convert_latitude, 3),
        ("latitude_direction", str, 4),
        ("longitude", convert_longitude, 5),
        ("longitude_direction", str, 6),
        ("speed", convert_knots_to_mps, 7),
        ("true_course", convert_deg_to_rads, 8),
    ],
    "GST": [
        ("utc_time", convert_time, 1),
        ("ranges_std_dev", safe_float, 2),
        ("semi_major_ellipse_std_dev", safe_float, 3),
        ("semi_minor_ellipse_std_dev", safe_float, 4),
        ("semi_major_orientation", safe_float, 5),
        ("lat_std_dev", safe_float, 6),
        ("lon_std_dev", safe_float, 7),
        ("alt_std_dev", safe_float, 8),
    ],
    "HDT": [
        ("heading", safe_float, 1),
    ],
    "VTG": [
        ("true_course", convert_deg_to_rads, 1),
        ("speed", convert_knots_to_mps, 5)
    ]
}

def parse_nmea_sentence(nmea_sentence):
    """Parse NMEA sentence into structured data"""
    if not nmea_sentence.strip():
        return None
    
    fields = [field.strip(',') for field in nmea_sentence.split(',')]
    
    if len(fields) < 1:
        return None
    
    # Extract sentence type (ignore $ and talker ID)
    if len(fields[0]) < 4:
        return None
    
    sentence_type = fields[0][3:]
    
    if sentence_type not in PARSE_MAPS:
        return None
    
    parse_map = PARSE_MAPS[sentence_type]
    parsed_sentence = {}
    
    try:
        for entry in parse_map:
            if entry[2] < len(fields):
                parsed_sentence[entry[0]] = entry[1](fields[entry[2]])
            else:
                # Use default values for missing fields
                if entry[1] == safe_float:
                    parsed_sentence[entry[0]] = float('NaN')
                elif entry[1] == safe_int:
                    parsed_sentence[entry[0]] = 0
                else:
                    parsed_sentence[entry[0]] = None
    except (ValueError, IndexError, TypeError):
        return None
    
    return {sentence_type: parsed_sentence}

class NmeaGpsCanConverter:
    """
    NMEA GPS to CAN Direct Converter (Pure Python Version)
    
    This class reads NMEA data from a serial port, parses GPS information,
    and directly sends it to CAN bus without ROS2.
    """
    
    def __init__(self, serial_port='/dev/ttyUSB0', serial_baudrate=115200, 
                 can_channel='can0', longitude_range=121.0, latitude_range=25.0):
        
        # Setup logging
        self.logger = logging.getLogger('nmea_gps_can_converter')
        
        # Configuration
        self.serial_port = serial_port
        self.serial_baudrate = serial_baudrate
        self.can_channel = can_channel
        self.longitude_range = longitude_range
        self.latitude_range = latitude_range
        
        # Control flags
        self.running = False
        self.shutdown_requested = False
        
        # Initialize CAN bus
        try:
            self.can_bus = can.interface.Bus(channel=self.can_channel, bustype='socketcan')
            self.logger.info(f"CAN bus initialized on channel: {self.can_channel}")
        except Exception as e:
            self.logger.error(f"Failed to initialize CAN bus: {e}")
            raise
        
        # Initialize serial connection
        try:
            self.serial_connection = serial.Serial(
                self.serial_port, 
                self.serial_baudrate, 
                timeout=1.0
            )
            self.logger.info(f"Serial connection established: {self.serial_port} @ {self.serial_baudrate}")
        except Exception as e:
            self.logger.error(f"Failed to open serial port: {e}")
            raise
        
        # GPS state variables
        self.last_latitude = 0.0
        self.last_longitude = 0.0
        self.position_valid = False
        self.current_heading = 0.0
        self.current_speed = 0.0
        
        # GPS quality mapping
        self.gps_qualities = {
            -1: 1000000.0,  # Unknown
            0: 1000000.0,   # Invalid
            1: 4.0,         # SPS
            2: 0.1,         # DGPS
            4: 0.02,        # RTK Fix
            5: 4.0,         # RTK Float
            9: 3.0,         # WAAS
        }
        
        self.logger.info("NMEA GPS to CAN Converter initialized successfully")
        self.logger.info(f"Serial: {self.serial_port} @ {self.serial_baudrate}")
        self.logger.info(f"CAN: {self.can_channel}")
        self.logger.info("Ready to process GPS data directly to CAN bus")
    
    def start(self):
        """Start the converter"""
        self.running = True
        
        # Start timestamp timer thread
        self.timestamp_thread = threading.Thread(target=self.timestamp_loop, daemon=True)
        self.timestamp_thread.start()
        
        # Start NMEA reading thread
        self.nmea_thread = threading.Thread(target=self.nmea_reader_loop, daemon=True)
        self.nmea_thread.start()
        
        self.logger.info("NMEA GPS to CAN Converter started")
        
        # Main loop
        try:
            while self.running and not self.shutdown_requested:
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.logger.info("Received interrupt signal, shutting down...")
            self.stop()
    
    def stop(self):
        """Stop the converter"""
        self.running = False
        self.shutdown_requested = True
        
        try:
            if hasattr(self, 'serial_connection') and self.serial_connection.is_open:
                self.serial_connection.close()
                self.logger.info("Serial connection closed")
        except Exception as e:
            self.logger.error(f"Error closing serial connection: {e}")
        
        self.logger.info("NMEA GPS to CAN Converter stopped")
    
    def timestamp_loop(self):
        """Send timestamp to CAN bus every second"""
        while self.running:
            try:
                self.send_timestamp()
                time.sleep(1.0)
            except Exception as e:
                self.logger.error(f"Error in timestamp loop: {e}")
                time.sleep(1.0)
    
    def send_timestamp(self):
        """Send timestamp to CAN bus every second"""
        try:
            # Calculate timestamp
            now = time.time()
            ms_since_midnight = int((now % 86400) * 1000)
            days_since_1984 = int((now - 441763200) // 86400)
            
            # Prepare data (little endian)
            timestamp_data = ms_since_midnight.to_bytes(4, 'little') + days_since_1984.to_bytes(2, 'little')
            
            # Send CAN message
            can_msg = can.Message(
                arbitration_id=0x100, 
                data=timestamp_data, 
                is_extended_id=False
            )
            self.can_bus.send(can_msg)
            
            current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            self.logger.debug(
                f'Timestamp sent (0x100) - Time: {current_time}, '
                f'MS since midnight: {ms_since_midnight}, Days since 1984: {days_since_1984}'
            )
            
        except can.CanError as e:
            self.logger.error(f'CAN Error sending timestamp: {e}')
        except Exception as e:
            self.logger.error(f'Error sending timestamp: {e}')
    
    def nmea_reader_loop(self):
        """Main loop for reading and processing NMEA data"""
        self.logger.info("NMEA reader loop started")
        
        while self.running:
            try:
                # Read line from serial port
                if not self.serial_connection.is_open:
                    self.logger.warning("Serial connection lost, attempting to reconnect...")
                    time.sleep(1.0)
                    continue
                
                line = self.serial_connection.readline().decode('ascii', errors='ignore').strip()
                
                if line:
                    self.process_nmea_sentence(line)
                    
            except serial.SerialException as e:
                self.logger.error(f"Serial error: {e}")
                time.sleep(1.0)
            except UnicodeDecodeError as e:
                self.logger.debug(f"Unicode decode error: {e}")
            except Exception as e:
                self.logger.error(f"Unexpected error in NMEA reader: {e}")
                time.sleep(0.1)
    
    def process_nmea_sentence(self, nmea_sentence):
        """Process a single NMEA sentence"""
        if not nmea_sentence.startswith('$'):
            return
        
        # Verify checksum
        if not check_nmea_checksum(nmea_sentence):
            self.logger.debug(f"Invalid checksum: {nmea_sentence}")
            return
        
        # Parse sentence
        parsed = parse_nmea_sentence(nmea_sentence)
        if not parsed:
            return
        
        # Send raw NMEA string to CAN
        self.send_nmea_string_to_can(nmea_sentence)
        
        # Process parsed data
        if 'GGA' in parsed:
            self.process_gga_data(parsed['GGA'])
        elif 'RMC' in parsed:
            self.process_rmc_data(parsed['RMC'])
        elif 'VTG' in parsed:
            self.process_vtg_data(parsed['VTG'])
        elif 'GST' in parsed:
            self.process_gst_data(parsed['GST'])
        elif 'HDT' in parsed:
            self.process_hdt_data(parsed['HDT'])
    
    def process_gga_data(self, data):
        """Process GGA (Global Positioning System Fix Data) sentence"""
        try:
            # Extract position data
            latitude = data.get('latitude', float('NaN'))
            longitude = data.get('longitude', float('NaN'))
            altitude = data.get('altitude', float('NaN'))
            mean_sea_level = data.get('mean_sea_level', 0.0)
            
            # Check if position data is valid
            if math.isnan(latitude) or math.isnan(longitude):
                return
            
            # Apply direction corrections
            if data.get('latitude_direction') == 'S':
                latitude = -latitude
            if data.get('longitude_direction') == 'W':
                longitude = -longitude
            
            # Validate position against expected range
            if not self.position_valid:
                if (abs(latitude - self.latitude_range) < 1.0 and 
                    abs(longitude - self.longitude_range) < 1.0):
                    self.last_latitude = latitude
                    self.last_longitude = longitude
                    self.position_valid = True
                else:
                    self.logger.debug(f"Position out of range: {latitude}, {longitude}")
                    return
            
            # Apply position filtering
            if abs(latitude - self.latitude_range) > 1.0:
                latitude = self.last_latitude
            else:
                self.last_latitude = latitude
            
            if abs(longitude - self.longitude_range) > 1.0:
                longitude = self.last_longitude
            else:
                self.last_longitude = longitude
            
            # Calculate altitude above sea level
            if not math.isnan(altitude) and not math.isnan(mean_sea_level):
                altitude_asl = altitude + mean_sea_level
            else:
                altitude_asl = 0.0
            
            # Get quality information
            fix_type = data.get('fix_type', 0)
            hdop = data.get('hdop', 1.0)
            num_satellites = data.get('num_satellites', 0)
            
            # Send GPS data to CAN
            self.send_gps_data_to_can(latitude, longitude, altitude_asl, fix_type, hdop, num_satellites)
            
        except Exception as e:
            self.logger.error(f"Error processing GGA data: {e}")
    
    def process_rmc_data(self, data):
        """Process RMC (Recommended Minimum) sentence"""
        try:
            if not data.get('fix_valid', False):
                return
            
            speed = data.get('speed', 0.0)  # Already in m/s
            course = data.get('true_course', 0.0)  # Already in radians
            
            if not math.isnan(speed) and not math.isnan(course):
                self.current_speed = speed
                self.current_heading = course
                self.send_velocity_data_to_can(speed, course)
                
        except Exception as e:
            self.logger.error(f"Error processing RMC data: {e}")
    
    def process_vtg_data(self, data):
        """Process VTG (Track Made Good and Ground Speed) sentence"""
        try:
            if not self.position_valid:
                return
            
            speed = data.get('speed', 0.0)  # Already in m/s
            course = data.get('true_course', 0.0)  # Already in radians
            
            if not math.isnan(speed) and not math.isnan(course):
                self.current_speed = speed
                self.current_heading = course
                self.send_velocity_data_to_can(speed, course)
                
        except Exception as e:
            self.logger.error(f"Error processing VTG data: {e}")
    
    def process_gst_data(self, data):
        """Process GST (GPS Pseudorange Noise Statistics) sentence"""
        # GST data could be used for error estimation in future versions
        pass
    
    def process_hdt_data(self, data):
        """Process HDT (Heading - True) sentence"""
        try:
            heading = data.get('heading', float('NaN'))
            if not math.isnan(heading):
                self.current_heading = math.radians(heading)
                
        except Exception as e:
            self.logger.error(f"Error processing HDT data: {e}")
    
    def send_gps_data_to_can(self, latitude, longitude, altitude, fix_type, hdop, num_satellites):
        """Send GPS position data to CAN bus"""
        try:
            # Encode GPS basic information (0x400)
            lat_scaled = int(latitude * 10**7)
            lon_scaled = int(longitude * 10**7)
            
            lat_bytes = struct.pack('<i', lat_scaled)
            lon_bytes = struct.pack('<i', lon_scaled)
            gps_basic_data = lat_bytes + lon_bytes
            
            # Encode altitude (0x401)
            height_scaled = int(altitude) if not math.isinf(altitude) else 0
            height_bytes = struct.pack('<h', height_scaled) + b'\x00' * 6
            
            # Encode GPS status (0x402)
            status_data = struct.pack('<BBBBxxxx', 
                                    fix_type, 
                                    num_satellites, 
                                    int(hdop * 10), 
                                    1 if self.position_valid else 0)
            
            # Create CAN messages
            messages = [
                can.Message(arbitration_id=0x400, data=gps_basic_data, is_extended_id=False),
                can.Message(arbitration_id=0x401, data=height_bytes, is_extended_id=False),
                can.Message(arbitration_id=0x402, data=status_data, is_extended_id=False),
            ]
            
            # Send all messages
            for msg in messages:
                self.can_bus.send(msg)
            
            self.logger.info(
                f"GPS CAN sent: Lat={latitude:.6f}, Lon={longitude:.6f}, "
                f"Alt={altitude:.1f}m, Fix={fix_type}, Sats={num_satellites}"
            )
            
        except Exception as e:
            self.logger.error(f"Failed to send GPS data to CAN: {e}")
    
    def send_velocity_data_to_can(self, speed, course):
        """Send velocity data to CAN bus"""
        try:
            # Calculate velocity components
            vx = speed * math.cos(course)  # m/s
            vy = speed * math.sin(course)  # m/s
            vz = 0.0  # Assume no vertical velocity
            
            # Scale to mm/s and pack
            vx_scaled = int(vx * 1000)
            vy_scaled = int(vy * 1000)
            vz_scaled = int(vz * 1000)
            speed_scaled = int(speed * 1000)
            
            vx_bytes = struct.pack('<i', vx_scaled) + b'\x00' * 4
            vy_bytes = struct.pack('<i', vy_scaled) + b'\x00' * 4
            vz_bytes = struct.pack('<i', vz_scaled) + b'\x00' * 4
            speed_bytes = struct.pack('<i', speed_scaled) + b'\x00' * 4
            
            # Create CAN messages for velocity (0x403-0x406)
            messages = [
                can.Message(arbitration_id=0x403, data=vx_bytes, is_extended_id=False),
                can.Message(arbitration_id=0x404, data=vy_bytes, is_extended_id=False),
                can.Message(arbitration_id=0x405, data=vz_bytes, is_extended_id=False),
                can.Message(arbitration_id=0x406, data=speed_bytes, is_extended_id=False),
            ]
            
            # Send all messages
            for msg in messages:
                self.can_bus.send(msg)
            
            speed_kmh = speed * 3.6
            course_deg = math.degrees(course)
            
            self.logger.info(
                f"Velocity CAN sent: Speed={speed_kmh:.2f}km/h, "
                f"Course={course_deg:.1f}°, VX={vx:.3f}m/s, VY={vy:.3f}m/s"
            )
            
        except Exception as e:
            self.logger.error(f"Failed to send velocity data to CAN: {e}")
    
    def send_nmea_string_to_can(self, nmea_string):
        """Send raw NMEA string to CAN bus for logging/debugging"""
        try:
            # Convert NMEA string to bytes
            nmea_bytes = nmea_string.encode('ascii')
            
            # Calculate frame count (8 bytes per frame)
            frame_count = (len(nmea_bytes) + 7) // 8
            
            # Send header frame (0x500)
            header_data = struct.pack('<HH', len(nmea_bytes), frame_count) + b'\x00' * 4
            header_msg = can.Message(arbitration_id=0x500, data=header_data, is_extended_id=False)
            self.can_bus.send(header_msg)
            
            # Send data frames (0x501+)
            for i in range(frame_count):
                start_idx = i * 8
                end_idx = min(start_idx + 8, len(nmea_bytes))
                frame_data = nmea_bytes[start_idx:end_idx]
                
                # Pad to 8 bytes if necessary
                if len(frame_data) < 8:
                    frame_data += b'\x00' * (8 - len(frame_data))
                
                data_msg = can.Message(
                    arbitration_id=0x501 + i, 
                    data=frame_data, 
                    is_extended_id=False
                )
                self.can_bus.send(data_msg)
            
            # Parse sentence type for logging
            sentence_type = "Unknown"
            if nmea_string.startswith('$') and ',' in nmea_string:
                parts = nmea_string.split(',')
                if len(parts[0]) >= 4:
                    sentence_type = parts[0][3:]
            
            self.logger.debug(
                f"NMEA CAN sent: Type={sentence_type}, "
                f"Length={len(nmea_bytes)} bytes, Frames={frame_count+1}"
            )
            
        except Exception as e:
            self.logger.error(f"Failed to send NMEA string to CAN: {e}")

def setup_logging(log_level=logging.INFO):
    """Setup logging configuration"""
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler('nmea_gps_can.log')
        ]
    )

def signal_handler(signum, frame):
    """Handle shutdown signals"""
    print("\nReceived shutdown signal, stopping...")
    sys.exit(0)

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='NMEA GPS to CAN Direct Converter')
    parser.add_argument('--serial-port', default='/dev/ttyUSB0', 
                       help='Serial port for GPS receiver (default: /dev/ttyUSB0)')
    parser.add_argument('--serial-baudrate', type=int, default=115200,
                       help='Serial port baudrate (default: 115200)')
    parser.add_argument('--can-channel', default='can0',
                       help='CAN channel name (default: can0)')
    parser.add_argument('--longitude-range', type=float, default=121.0,
                       help='Expected longitude range for position validation (default: 121.0)')
    parser.add_argument('--latitude-range', type=float, default=25.0,
                       help='Expected latitude range for position validation (default: 25.0)')
    parser.add_argument('--log-level', default='INFO',
                       choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                       help='Logging level (default: INFO)')
    
    args = parser.parse_args()
    
    # Setup logging
    log_level = getattr(logging, args.log_level.upper())
    setup_logging(log_level)
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    logger = logging.getLogger('main')
    logger.info("Starting NMEA GPS to CAN Direct Converter")
    
    try:
        # Create converter
        converter = NmeaGpsCanConverter(
            serial_port=args.serial_port,
            serial_baudrate=args.serial_baudrate,
            can_channel=args.can_channel,
            longitude_range=args.longitude_range,
            latitude_range=args.latitude_range
        )
        
        # Start converter
        converter.start()
        
    except KeyboardInterrupt:
        logger.info("Received interrupt signal, shutting down...")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        if 'converter' in locals():
            converter.stop()
        logger.info("NMEA GPS to CAN Converter terminated")

if __name__ == '__main__':
    main()
