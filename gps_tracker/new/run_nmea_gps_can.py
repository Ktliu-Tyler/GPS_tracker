#!/usr/bin/env python3
"""
Standalone NMEA GPS to CAN Converter
Run this script directly without ROS2 environment
"""

import sys
import os

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from nmea_gps_can import main

if __name__ == '__main__':
    main()
