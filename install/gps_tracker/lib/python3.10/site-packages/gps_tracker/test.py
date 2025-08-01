import csv
import struct
import re

def parse_lat_lon(lat_str, lat_dir, lon_str, lon_dir):
    # 緯度處理
    lat_deg = int(float(lat_str) / 100)
    lat_min = float(lat_str) - lat_deg * 100
    latitude = lat_deg + lat_min / 60.0
    if lat_dir == 'S':
        latitude = -latitude
    
    # 經度處理
    lon_deg = int(float(lon_str) / 100)
    lon_min = float(lon_str) - lon_deg * 100
    longitude = lon_deg + lon_min / 60.0
    if lon_dir == 'W':
        longitude = -longitude
    
    return latitude, longitude

def nmea_to_csv(input_lines, output_file):
    writer = csv.writer(open(output_file, 'w', newline=''))
    writer.writerow(["Timestamp", "ID", "Extended", "Bus", "Length", "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7"])

    timestamp = 0.0
    for line in input_lines:
        if "$GNRMC" in line:
            parts = line.strip().split(",")
            if len(parts) > 6 and parts[2] == "A":
                time_str = parts[1]
                lat_str, lat_dir = parts[3], parts[4]
                lon_str, lon_dir = parts[5], parts[6]
                lat, lon = parse_lat_lon(lat_str, lat_dir, lon_str, lon_dir)
                timestamp += 0.01  # 每筆加 0.01 秒

                # 經度 CAN frame
                lon_bytes = struct.pack('<f', lon)
                lon_row = [f"{timestamp:.6f}", "257", "0", "0", "4"] + list(lon_bytes)
                writer.writerow(lon_row)

                # 緯度 CAN frame
                lat_bytes = struct.pack('<f', lat)
                lat_row = [f"{timestamp:.6f}", "256", "0", "0", "4"] + list(lat_bytes)
                writer.writerow(lat_row)

input_data = [
    
]

nmea_to_csv(input_data, "gps_can_output.csv")
