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

import math

import rclpy

import socket
import sys
import datetime
import base64
import time
import serial
import ssl
from pynmeagps import NMEAReader

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from transformations import quaternion_from_euler
from gps_tracker.checksum_utils import check_nmea_checksum
from gps_tracker import parser
from std_msgs.msg import String


class Ros2NMEADriver(Node):
    def __init__(self, longtitude_range = 121, latitude_range = 25):
        super().__init__('nturt_nmea_navsat_driver')

        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, 'vel', 10)
        self.heading_pub = self.create_publisher(QuaternionStamped, 'heading', 10)
        self.time_ref_pub = self.create_publisher(TimeReference, 'time_reference', 10)

        self.time_ref_source = self.declare_parameter('time_ref_source', 'gps').value
        self.use_RMC = self.declare_parameter('useRMC', True).value
        self.valid_fix = False

        # epe = estimated position error
        self.default_epe_quality0 = self.declare_parameter('epe_quality0', 1000000).value
        self.default_epe_quality1 = self.declare_parameter('epe_quality1', 4.0).value
        self.default_epe_quality2 = self.declare_parameter('epe_quality2', 0.1).value
        self.default_epe_quality4 = self.declare_parameter('epe_quality4', 0.02).value
        self.default_epe_quality5 = self.declare_parameter('epe_quality5', 4.0).value
        self.default_epe_quality9 = self.declare_parameter('epe_quality9', 3.0).value

        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        self.long_range = longtitude_range
        self.lat_range = latitude_range
        self.last_long = 0
        self.last_lat = 0
        self.valid = True

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if not check_nmea_checksum(nmea_string):
            self.get_logger().debug("Received a sentence with an invalid checksum. " +
                                   "Sentence was: %s" % nmea_string)
            return False  # Skip invalid checksum messages

        parsed_sentence = parser.parse_nmea_sentence(nmea_string)

        if not parsed_sentence:
            self.get_logger().debug("Failed to parse NMEA sentence. Sentence was: %s" % nmea_string)
            return False

        try:
            if timestamp:
                current_time = timestamp
            else:
                current_time = self.get_clock().now().to_msg()

            current_fix = NavSatFix()
            current_fix.header.stamp = current_time
            current_fix.header.frame_id = frame_id
            current_time_ref = TimeReference()
            current_time_ref.header.stamp = current_time
            current_time_ref.header.frame_id = frame_id
            if self.time_ref_source:
                current_time_ref.source = self.time_ref_source
            else:
                current_time_ref.source = frame_id

            if 'GGA' in parsed_sentence:
                current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                data = parsed_sentence['GGA']
                fix_type = data['fix_type']
                if not (fix_type in self.gps_qualities):
                    fix_type = -1
                gps_qual = self.gps_qualities[fix_type]
                default_epe = gps_qual[0]
                current_fix.status.status = gps_qual[1]
                current_fix.position_covariance_type = gps_qual[2]
                if current_fix.status.status > 0:
                    self.valid_fix = True
                else:
                    self.valid_fix = False

                current_fix.status.service = NavSatStatus.SERVICE_GPS
                latitude = data['latitude']
                longitude = data['longitude']

                # Ensure latitude and longitude are valid numbers
                if latitude is None or longitude is None:
                    return False
                
                latitude = float(latitude)
                longitude = float(longitude)

                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                if data['longitude_direction'] == 'W':
                    longitude = -longitude

                if not self.valid and abs(latitude - self.lat_range) < 1 and abs(longitude - self.long_range) < 1:
                    self.last_lat = latitude
                    self.last_long = longitude
                    self.valid = True
                
                if not self.valid:
                    return

                if abs(latitude - self.lat_range) > 1:
                    latitude = self.last_lat
                current_fix.latitude = float(latitude)
                self.last_lat = latitude

                if abs(longitude - self.long_range) > 1:
                    longitude = self.last_long
                current_fix.longitude = float(longitude)
                self.last_long = longitude

                # Altitude is above ellipsoid, so adjust for mean-sea-level
                altitude = data['altitude'] + data['mean_sea_level']
                current_fix.altitude = float(altitude)

                # use default epe std_dev unless we've received a GST sentence with epes
                if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                    self.lon_std_dev = default_epe
                if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                    self.lat_std_dev = default_epe
                if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                    self.alt_std_dev = default_epe * 2

                hdop = data['hdop']
                if hdop is None:
                    hdop = 1.0
                else:
                    hdop = float(hdop)
                    
                current_fix.position_covariance[0] = float((hdop * self.lon_std_dev) ** 2)
                current_fix.position_covariance[4] = float((hdop * self.lat_std_dev) ** 2)
                current_fix.position_covariance[8] = float((2 * hdop * self.alt_std_dev) ** 2)  # FIXME

                self.fix_pub.publish(current_fix)

                if not math.isnan(data['utc_time']):
                    current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                    self.last_valid_fix_time = current_time_ref
                    self.time_ref_pub.publish(current_time_ref)

            elif not self.use_RMC and 'VTG' in parsed_sentence:
                data = parsed_sentence['VTG']

                # Only report VTG data when you've received a valid GGA fix as well.
                if self.valid_fix:
                    speed = data['speed']
                    true_course = data['true_course']
                    
                    # Ensure speed and course are valid numbers
                    if speed is not None and true_course is not None:
                        speed = float(speed)
                        true_course = float(true_course)
                        
                        current_vel = TwistStamped()
                        current_vel.header.stamp = current_time
                        current_vel.header.frame_id = frame_id
                        current_vel.twist.linear.x = speed * math.sin(true_course)
                        current_vel.twist.linear.y = speed * math.cos(true_course)
                        self.vel_pub.publish(current_vel)

            elif 'RMC' in parsed_sentence:
                data = parsed_sentence['RMC']

                # Only publish a fix from RMC if the use_RMC flag is set.
                if self.use_RMC:
                    if data['fix_valid']:
                        current_fix.status.status = NavSatStatus.STATUS_FIX
                    else:
                        current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                    current_fix.status.service = NavSatStatus.SERVICE_GPS
                    latitude = data['latitude']
                    longitude = data['longitude']

                    # Ensure latitude and longitude are valid numbers
                    if latitude is None or longitude is None:
                        return False
                    
                    latitude = float(latitude)
                    longitude = float(longitude)

                    if data['latitude_direction'] == 'S':
                        latitude = -latitude
                    if data['longitude_direction'] == 'W':
                        longitude = -longitude

                    if not self.valid and abs(latitude - self.lat_range) < 1 and abs(longitude - self.long_range) < 1:
                        self.last_lat = latitude
                        self.last_long = longitude
                        self.valid = True
                    
                    if not self.valid:
                        return

                    if abs(latitude - self.lat_range) > 1:
                        latitude = self.last_lat
                    current_fix.latitude = float(latitude)
                    self.last_lat = latitude

                    if abs(longitude - self.long_range) > 1:
                        longitude = self.last_long
                    current_fix.longitude = float(longitude)
                    self.last_long = longitude

                    current_fix.altitude = 0.0
                    current_fix.position_covariance_type = \
                        NavSatFix.COVARIANCE_TYPE_UNKNOWN

                    self.fix_pub.publish(current_fix)

                    if not math.isnan(data['utc_time']):
                        current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                        self.time_ref_pub.publish(current_time_ref)

                # Publish velocity from RMC regardless, since GGA doesn't provide it.
                if data['fix_valid']:
                    speed = data['speed']
                    true_course = data['true_course']
                    
                    # Ensure speed and course are valid numbers
                    if speed is not None and true_course is not None:
                        speed = float(speed)
                        true_course = float(true_course)
                        
                        current_vel = TwistStamped()
                        current_vel.header.stamp = current_time
                        current_vel.header.frame_id = frame_id
                        current_vel.twist.linear.x = speed * math.sin(true_course)
                        current_vel.twist.linear.y = speed * math.cos(true_course)
                        self.vel_pub.publish(current_vel)
            elif 'GST' in parsed_sentence:
                data = parsed_sentence['GST']

                # Use receiver-provided error estimate if available
                self.using_receiver_epe = True
                self.lon_std_dev = data['lon_std_dev']
                self.lat_std_dev = data['lat_std_dev']
                self.alt_std_dev = data['alt_std_dev']
            elif 'HDT' in parsed_sentence:
                data = parsed_sentence['HDT']
                if data['heading']:
                    current_heading = QuaternionStamped()
                    current_heading.header.stamp = current_time
                    current_heading.header.frame_id = frame_id
                    q = quaternion_from_euler(0, 0, math.radians(data['heading']))
                    current_heading.quaternion.x = q[0]
                    current_heading.quaternion.y = q[1]
                    current_heading.quaternion.z = q[2]
                    current_heading.quaternion.w = q[3]
                    self.heading_pub.publish(current_heading)
            else:
                return False
            return True
        
        except (ValueError, TypeError, AttributeError) as e:
            self.get_logger().debug(f"Error processing NMEA sentence: {e}. Sentence: {nmea_string}")
            return False

    """Helper method for getting the frame_id with the correct TF prefix"""
    def get_frame_id(self):
        frame_id = self.declare_parameter('frame_id', 'gps').value
        prefix = self.declare_parameter('tf_prefix', '').value
        if len(prefix):
            return '%s/%s' % (prefix, frame_id)
        return frame_id

version=0.2
useragent="NTRIP JCMBsoftPythonClient/%.1f" % version

# reconnect parameter (fixed values):
factor=2 # How much the sleep time increases with each failed attempt
maxReconnect=1
maxReconnectTime=1200
sleepTime=1 # So the first one is 1 second

class NtripClient(object):
    def __init__(self,
                 buffer=50,
                 user="",
                 out=sys.stdout,
                 port=2101,
                 caster="",
                 mountpoint="",
                 host=False,
                 lat=46,
                 lon=122,
                 height=1212,
                 ssl=False,
                 verbose=False,
                 V2=False,
                 headerFile=sys.stderr,
                 headerOutput=False,
                 maxConnectTime=0
                 ):
        self.buffer=buffer
        self.user=base64.b64encode(bytes(user,'utf-8')).decode("utf-8")
#        print(self.user)
        self.out=out
        self.port=port
        self.caster=caster
        self.mountpoint=mountpoint
        self.setPosition(lat, lon)
        self.height=height
        self.verbose=verbose
        self.ssl=ssl
        self.host=host
        self.V2=V2
        self.headerFile=headerFile
        self.headerOutput=headerOutput
        self.maxConnectTime=maxConnectTime
        self.socket=None
        self.stream = serial.Serial('/dev/ttyUSB0', 115200, timeout=3)
        self.nmr = NMEAReader(self.stream)
        self.driver = Ros2NMEADriver()
        self.frame_id = self.driver.get_frame_id()


    def setPosition(self, lat, lon):
        self.flagN="N"
        self.flagE="E"
        if lon>180:
            lon=(lon-360)*-1
            self.flagE="W"
        elif (lon<0 and lon>= -180):
            lon=lon*-1
            self.flagE="W"
        elif lon<-180:
            lon=lon+360
            self.flagE="E"
        else:
            self.lon=lon
        if lat<0:
            lat=lat*-1
            self.flagN="S"
        self.lonDeg=int(lon)
        self.latDeg=int(lat)
        self.lonMin=(lon-self.lonDeg)*60
        self.latMin=(lat-self.latDeg)*60

    def getMountPointBytes(self):
        mountPointString = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (self.mountpoint, useragent, self.user)
#        mountPointString = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\n" % (self.mountpoint, useragent)
        if self.host or self.V2:
           hostString = "Host: %s:%i\r\n" % (self.caster,self.port)
           mountPointString+=hostString
        if self.V2:
           mountPointString+="Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString+="\r\n"
        if self.verbose:
           self.driver.get_logger().debug(mountPointString)
        return bytes(mountPointString,'ascii')

    def getGGABytes(self):
        while True:
            try:
                (raw_data, parsed_data) = self.nmr.read()
                if bytes("GNGGA",'ascii') in raw_data :
                    # print(parsed_data)
                    return raw_data
            except (serial.SerialException, UnicodeDecodeError) as e:
                if self.verbose:
                    self.driver.get_logger().debug(f"Error reading NMEA data: {e}")
                continue

    def readData(self):
        reconnectTry=1
        sleepTime=1
        reconnectTime=0
        if self.maxConnectTime > 0 :
            EndConnect=datetime.timedelta(seconds=self.maxConnectTime)
        try:
            while reconnectTry<=maxReconnect:
                found_header=False
                if self.verbose:
                    self.driver.get_logger().debug('Connection {0} of {1}\n'.format(reconnectTry,maxReconnect))

                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                if self.ssl:
                    self.socket=ssl.wrap_socket(self.socket)

                error_indicator = self.socket.connect_ex((self.caster, self.port))
                if error_indicator==0:
                    sleepTime = 1
                    connectTime=datetime.datetime.now()

                    self.socket.settimeout(10)
                    self.socket.sendall(self.getMountPointBytes())
                    while not found_header:
                        casterResponse=self.socket.recv(4096) #All the data
                        # print(casterResponse)
                        try:
                            header_lines = casterResponse.decode('utf-8').split("\r\n")
                        except UnicodeDecodeError:
                            self.driver.get_logger().debug("header not found yet, error occured, retrying....")
                            continue
                        
# header_lines empty, request fail,exit while loop
                        for line in header_lines:
                            if line=="":
                                if not found_header:
                                    found_header=True
                                    if self.verbose:
                                        self.driver.get_logger().debug("End Of Header"+"\n")
                            else:
                                if self.verbose:
                                    self.driver.get_logger().debug("Header: " + line+"\n")
                            if self.headerOutput:
                                self.headerFile.write(line+"\n")



#header_lines has content
                        for line in header_lines:
                            if line.find("SOURCETABLE")>=0:
                                self.driver.get_logger().error("Mount point does not exist")
                                sys.exit(1)
                            elif line.find("401 Unauthorized")>=0:
                                self.driver.get_logger().error("Unauthorized request\n")
                                sys.exit(1)
                            elif line.find("404 Not Found")>=0:
                                self.driver.get_logger().error("Mount Point does not exist\n")
                                sys.exit(2)
                            elif line.find("ICY 200 OK")>=0:
                                #Request was valid
                                self.socket.sendall(self.getGGABytes())
                            elif line.find("HTTP/1.0 200 OK")>=0:
                                #Request was valid
                                self.socket.sendall(self.getGGABytes())
                            elif line.find("HTTP/1.1 200 OK")>=0:
                                #Request was valid
                                self.socket.sendall(self.getGGABytes())



                    data = "Initial data"
                    while data:
                        try:
                            data=self.socket.recv(self.buffer)
                            # self.out.buffer.write(data)
                            self.stream.write(data)
                            try:
                                (raw_data, parsed_data) = self.nmr.read()
                                if bytes("GNGGA",'ascii') in raw_data or bytes("GNRMC", 'ascii') in raw_data:
                                    self.driver.get_logger().debug(raw_data.decode('utf-8'))
                                    self.driver.add_sentence(raw_data.decode('utf-8'), self.frame_id)
                            except (serial.SerialException, UnicodeDecodeError) as nmea_error:
                                if self.verbose:
                                    self.driver.get_logger().debug(f"Error reading NMEA data: {nmea_error}")
                                continue

#                            print datetime.datetime.now()-connectTime
                            if self.maxConnectTime :
                                if datetime.datetime.now() > connectTime+EndConnect:
                                    if self.verbose:
                                        self.driver.get_logger().error("Connection Timed exceeded\n")
                                    sys.exit(0)
#                            self.socket.sendall(self.getGGAString())



                        except socket.timeout:
                            if self.verbose:
                                self.driver.get_logger().debug('Connection TimedOut\n')
                            data=False
                        except socket.error:
                            if self.verbose:
                                self.driver.get_logger().debug('Connection Error\n')
                            data=False

                    if self.verbose:
                        self.driver.get_logger().debug('Closing Connection\n')
                    self.socket.close()
                    self.socket=None

                    if reconnectTry < maxReconnect :
                        self.driver.get_logger().info( "%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime *= factor

                        if sleepTime>maxReconnectTime:
                            sleepTime=maxReconnectTime
                    else:
                        sys.exit(1)


                    reconnectTry += 1
                else:
                    self.socket=None
                    if self.verbose:
                        self.driver.get_logger().debug(f"Error indicator: {error_indicator}")

                    if reconnectTry < maxReconnect :
                        self.driver.get_logger().info( "%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime *= factor
                        if sleepTime>maxReconnectTime:
                            sleepTime=maxReconnectTime
                    reconnectTry += 1

        except KeyboardInterrupt:
            if self.socket:
                self.socket.close()
            self.stream.close()
            sys.exit()