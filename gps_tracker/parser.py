# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import re
import time
import calendar
import math
import rclpy

logger = rclpy.logging.get_logger('nmea_navsat_driver')


def safe_float(field):
    try:
        return float(field)
    except ValueError:
        return float('NaN')


def safe_int(field):
    try:
        return int(field)
    except ValueError:
        return 0


def convert_latitude(field):
    return safe_float(field[0:2]) + safe_float(field[2:]) / 60.0


def convert_longitude(field):
    return safe_float(field[0:3]) + safe_float(field[3:]) / 60.0


def convert_time(nmea_utc):
    # Get current time in UTC for date information
    utc_struct = time.gmtime()  # immutable, so cannot modify this one
    utc_list = list(utc_struct)
    # If one of the time fields is empty, return NaN seconds
    if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6]:
        return float('NaN')
    else:
        hours = int(nmea_utc[0:2])
        minutes = int(nmea_utc[2:4])
        seconds = int(nmea_utc[4:6])
        utc_list[3] = hours
        utc_list[4] = minutes
        utc_list[5] = seconds
        unix_time = calendar.timegm(tuple(utc_list))
        return unix_time


def convert_status_flag(status_flag):
    if status_flag == "A":
        return True
    elif status_flag == "V":
        return False
    else:
        return False


def convert_knots_to_mps(knots):
    return safe_float(knots) * 0.514444444444


# Need this wrapper because math.radians doesn't auto convert inputs
def convert_deg_to_rads(degs):
    return math.radians(safe_float(degs))


"""Format for this dictionary is a sentence identifier (e.g. "GGA") as the key, with a
list of tuples where each tuple is a field name, conversion function and index
into the split sentence"""
parse_maps = {
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
    # Check for a valid nmea sentence

    # Not sure whether I should close this or not
    # if not re.match(r'(^\$GP|^\$GN|^\$GL|^\$IN).*\*[0-9A-Fa-f]{2}$', nmea_sentence):
    #     logger.debug("Regex didn't match, sentence not valid NMEA? Sentence was: %s"
    #                  % repr(nmea_sentence))
    #     return False
    fields = [field.strip(',') for field in nmea_sentence.split(',')]

    # Ignore the $ and talker ID portions (e.g. GP)
    sentence_type = fields[0][3:]

    if sentence_type not in parse_maps:
        logger.debug("Sentence type %s not in parse map, ignoring."
                     % repr(sentence_type))
        return False

    parse_map = parse_maps[sentence_type]

    parsed_sentence = {}
    for entry in parse_map:
        parsed_sentence[entry[0]] = entry[1](fields[entry[2]])

    return {sentence_type: parsed_sentence}

def process_nmea_sentence(nmea_sentence):
    """
    Process an NMEA sentence, distinguish between GNGGA and GNRMC,
    and return a human-readable string representation of the data.

    :param nmea_sentence: A string containing the NMEA sentence.
    :return: A formatted string describing the NMEA data.
    """
    if not nmea_sentence.startswith("$"):
        return "Invalid NMEA sentence: Does not start with '$'."

    # Split the sentence into fields
    fields = nmea_sentence.split(',')

    # Extract the sentence type (e.g., GNGGA or GNRMC)
    sentence_type = fields[0][1:]

    if sentence_type == "GNGGA":
        # Process GNGGA sentence
        try:
            time_utc = fields[1]
            latitude = float(fields[2])
            lat_direction = fields[3]
            longitude = float(fields[4])
            lon_direction = fields[5]
            fix_quality = int(fields[6])
            num_satellites = int(fields[7])
            hdop = float(fields[8])
            altitude = float(fields[9])
            altitude_units = fields[10]
            geoid_height = float(fields[11])
            geoid_units = fields[12]

            # Convert latitude and longitude to decimal degrees
            lat_degrees = int(latitude / 100)
            lat_minutes = latitude - (lat_degrees * 100)
            latitude_decimal = lat_degrees + (lat_minutes / 60.0)
            if lat_direction == 'S':
                latitude_decimal = -latitude_decimal

            lon_degrees = int(longitude / 100)
            lon_minutes = longitude - (lon_degrees * 100)
            longitude_decimal = lon_degrees + (lon_minutes / 60.0)
            if lon_direction == 'W':
                longitude_decimal = -longitude_decimal

            return (
                f"GNGGA Sentence:\n"
                f"  UTC Time: {time_utc}\n"
                f"  Latitude: {latitude_decimal}° {lat_direction}\n"
                f"  Longitude: {longitude_decimal}° {lon_direction}\n"
                f"  Fix Quality: {fix_quality}\n"
                f"  Number of Satellites: {num_satellites}\n"
                f"  HDOP: {hdop}\n"
                f"  Altitude: {altitude} {altitude_units}\n"
                f"  Geoid Height: {geoid_height} {geoid_units}\n"
            )
        except (ValueError, IndexError):
            logger.error("Invalid GNGGA sentence: %s" % nmea_sentence)
            return "Invalid GNGGA sentence: Missing or malformed fields."

    elif sentence_type == "GNRMC":
        # Process GNRMC sentence
        try:
            time_utc = fields[1]
            status = fields[2]
            latitude = float(fields[3])
            lat_direction = fields[4]
            longitude = float(fields[5])
            lon_direction = fields[6]
            speed = float(fields[7])
            course = float(fields[8])
            date = fields[9]

            # Convert latitude and longitude to decimal degrees
            lat_degrees = int(latitude / 100)
            lat_minutes = latitude - (lat_degrees * 100)
            latitude_decimal = lat_degrees + (lat_minutes / 60.0)
            if lat_direction == 'S':
                latitude_decimal = -latitude_decimal

            lon_degrees = int(longitude / 100)
            lon_minutes = longitude - (lon_degrees * 100)
            longitude_decimal = lon_degrees + (lon_minutes / 60.0)
            if lon_direction == 'W':
                longitude_decimal = -longitude_decimal

            return (
                f"GNRMC Sentence:\n"
                f"  UTC Time: {time_utc}\n"
                f"  Status: {'Valid' if status == 'A' else 'Invalid'}\n"
                f"  Latitude: {latitude_decimal}° {lat_direction}\n"
                f"  Longitude: {longitude_decimal}° {lon_direction}\n"
                f"  Speed: {speed} knots\n"
                f"  Course: {course}°\n"
                f"  Date: {date}\n"
            )
        except (ValueError, IndexError):
            return "Invalid GNRMC sentence: Missing or malformed fields."

    else:
        return f"Unsupported NMEA sentence type: {sentence_type}"


# Example usage
nmea_gngga = "$GNGGA,131639.000,2500.819365,N,12131.679105,E,2,34,0.58,13.7,M,15.2,M,0.2,0000*60"
nmea_gnrmc = "$GNRMC,131639.000,A,2500.819365,N,12131.679105,E,0.0,0.0,140525,,,A*6C"

print(process_nmea_sentence(nmea_gngga))
print(process_nmea_sentence(nmea_gnrmc))