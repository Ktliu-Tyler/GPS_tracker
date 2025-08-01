#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import struct
import math
import time
from datetime import datetime

# 嘗試導入CAN庫，如果失敗則提供備選方案
try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False
    print("⚠️  python-can 庫未安裝，僅顯示模擬資料")

class CANSystemTestPublisher(Node):
    """CAN系統假資料發送器"""
    def __init__(self):
        super().__init__('can_system_test_publisher')
        
        # 嘗試連接CAN匯流排
        if CAN_AVAILABLE:
            try:
                self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
                self.can_connected = True
                self.get_logger().info("✅ CAN匯流排連接成功")
            except Exception as e:
                self.can_connected = False
                self.get_logger().warn(f"⚠️  CAN匯流排連接失敗: {e}")
        else:
            self.can_connected = False
        
        # 建立定時器
        self.timer_timestamp = self.create_timer(1.0, self.send_timestamp)      # 1Hz 時間戳
        self.timer_battery = self.create_timer(0.2, self.send_battery_data)     # 5Hz 蓄電池
        self.timer_inverter = self.create_timer(0.1, self.send_inverter_data)   # 10Hz 逆變器
        self.timer_heartbeat = self.create_timer(1.0, self.send_heartbeat)      # 1Hz 心跳
        
        # 模擬資料狀態
        self.time_counter = 0.0
        self.battery_soc = 85.0     # 初始電量85%
        self.cell_index = 0         # 電池組索引
        self.temp_index = 0         # 溫度感測器索引
        
        self.get_logger().info("🔋 CAN系統測試發布器已啟動")
        self.get_logger().info("📡 發送蓄電池、逆變器、時間戳假資料")
        
        if not self.can_connected:
            self.get_logger().info("💻 模擬模式：僅顯示資料內容")
    
    def send_can_message(self, can_id: int, data: bytes, description: str = ""):
        """發送CAN訊息或顯示模擬資料"""
        if self.can_connected:
            try:
                msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
                self.bus.send(msg)
                self.get_logger().debug(f"✅ 發送CAN 0x{can_id:03X}: {description}")
            except Exception as e:
                self.get_logger().error(f"❌ CAN發送失敗 0x{can_id:03X}: {e}")
        else:
            hex_data = ' '.join([f'{b:02X}' for b in data])
            self.get_logger().info(f"📡 模擬CAN 0x{can_id:03X}: {hex_data} - {description}")
    
    def send_timestamp(self):
        """發送時間戳資料 (0x100)"""
        current_time = datetime.now()
        ms_since_midnight = int((current_time.hour * 3600 + current_time.minute * 60 + 
                                current_time.second) * 1000 + current_time.microsecond / 1000)
        days_since_1984 = int((current_time - datetime(1984, 1, 1)).days)
        
        # 6-byte 時間戳格式
        data = struct.pack('<IH', ms_since_midnight, days_since_1984)
        
        self.send_can_message(0x100, data, 
                            f"時間戳: {current_time.strftime('%H:%M:%S')}")
    
    def send_battery_data(self):
        """發送蓄電池系統資料"""
        # 1. Cell Voltage (0x190) - 電池電壓
        voltages = []
        for i in range(7):
            voltage_v = 3.6 + 0.3 * math.sin(self.time_counter + i * 0.5)  # 3.3-3.9V
            voltage_raw = int(voltage_v / 0.02)  # 20mV/LSB
            voltages.append(min(255, max(0, voltage_raw)))
        
        cell_data = struct.pack('<B', self.cell_index) + struct.pack('<7B', *voltages)
        self.send_can_message(0x190, cell_data, 
                            f"電池電壓組{self.cell_index}: {[v*0.02 for v in voltages][:3]}V...")
        
        # 2. Accumulator Status (0x290) - 蓄電池狀態
        status = 1  # OK
        temp_c = 25.0 + 15.0 * math.sin(self.time_counter * 0.3)
        temp_raw = int(temp_c / 0.125)
        voltage_v = 48.0 + 6.0 * math.sin(self.time_counter * 0.2)
        voltage_raw = int(voltage_v * 1024)
        
        status_data = struct.pack('<BhH', status, temp_raw, voltage_raw) + b'\x00\x00\x00'
        self.send_can_message(0x290, status_data, 
                            f"蓄電池狀態: OK, {temp_c:.1f}°C, {voltage_v:.1f}V")
        
        # 3. Temperature (0x390) - 溫度監測
        temperatures = []
        for i in range(7):
            temp_c = 30.0 + 8.0 * math.sin(self.time_counter + i * 0.3)
            temp_raw = int(temp_c / 0.5)  # 0.5°C/LSB
            temperatures.append(min(255, max(0, temp_raw)))
        
        temp_data = struct.pack('<B', self.temp_index) + struct.pack('<7B', *temperatures)
        self.send_can_message(0x390, temp_data, 
                            f"溫度組{self.temp_index}: {[t*0.5 for t in temperatures][:3]}°C...")
        
        # 4. State (0x490) - SOC、電流、容量
        self.battery_soc = max(10.0, min(100.0, self.battery_soc - 0.02))  # 緩慢放電
        current_a = 25.0 + 15.0 * math.sin(self.time_counter * 0.4)
        current_raw = int(current_a / 0.01)
        capacity_ah = 55.0
        capacity_raw = int(capacity_ah / 0.01)
        
        state_data = struct.pack('<Bhh', int(self.battery_soc), current_raw, capacity_raw) + b'\x00\x00\x00'
        self.send_can_message(0x490, state_data, 
                            f"電池狀態: SOC={self.battery_soc:.0f}%, {current_a:.1f}A, {capacity_ah:.1f}Ah")
        
        # 更新索引
        self.cell_index = (self.cell_index + 1) % 8
        self.temp_index = (self.temp_index + 1) % 8
    
    def send_inverter_data(self):
        """發送逆變器系統資料"""
        inverter_names = ['FL', 'FR', 'RL', 'RR']
        
        for i in range(1, 5):  # 1-4 對應 FL, FR, RL, RR
            # 1. Inverter Status (0x191-0x194)
            status_word = 0x1000 + i  # 模擬狀態字
            torque_nm = 80.0 + 40.0 * math.sin(self.time_counter + i * 0.5)
            torque_raw = int(torque_nm * 1000)  # 1/1000 rated torque/LSB
            speed_rpm = 1800 + 600 * math.sin(self.time_counter * 0.6 + i)
            speed_raw = int(speed_rpm)
            
            status_data = struct.pack('<Hhh', status_word, torque_raw, speed_raw) + b'\x00\x00'
            self.send_can_message(0x190 + i, status_data, 
                                f"逆變器{inverter_names[i-1]}狀態: {torque_nm:.1f}Nm, {speed_rpm:.0f}RPM")
            
            # 2. Inverter State (0x291-0x294) - 電源狀態
            dc_voltage_v = 320.0 + 30.0 * math.sin(self.time_counter + i * 0.3)
            dc_voltage_raw = int(dc_voltage_v * 100)
            dc_current_a = 60.0 + 25.0 * math.sin(self.time_counter + i * 0.7)
            dc_current_raw = int(dc_current_a * 100)
            
            state_data = struct.pack('<HH', dc_voltage_raw, dc_current_raw) + b'\x00\x00\x00\x00'
            self.send_can_message(0x290 + i, state_data, 
                                f"逆變器{inverter_names[i-1]}電源: {dc_voltage_v:.1f}V, {dc_current_a:.1f}A")
            
            # 3. Inverter Temperature (0x391-0x394) - 溫度
            mos_temp_c = 65.0 + 20.0 * math.sin(self.time_counter + i)
            mcu_temp_c = 50.0 + 15.0 * math.sin(self.time_counter + i * 0.5)
            motor_temp_c = 75.0 + 25.0 * math.sin(self.time_counter + i * 0.8)
            
            mos_temp_raw = int(mos_temp_c * 10)
            mcu_temp_raw = int(mcu_temp_c * 10)
            motor_temp_raw = int(motor_temp_c * 10)
            
            temp_data = struct.pack('<hhh', mos_temp_raw, mcu_temp_raw, motor_temp_raw) + b'\x00\x00'
            self.send_can_message(0x390 + i, temp_data, 
                                f"逆變器{inverter_names[i-1]}溫度: MOS={mos_temp_c:.0f}°C, MCU={mcu_temp_c:.0f}°C, 馬達={motor_temp_c:.0f}°C")
        
        self.time_counter += 0.1
    
    def send_heartbeat(self):
        """發送心跳信號"""
        # Accumulator Heartbeat (0x710)
        self.send_can_message(0x710, b'\x7F', "蓄電池心跳: OK")
        
        # Inverter Heartbeats (0x711-0x714)
        for i in range(1, 5):
            inverter_names = ['FL', 'FR', 'RL', 'RR']
            self.send_can_message(0x710 + i, b'\x7F', f"逆變器{inverter_names[i-1]}心跳: OK")

def main(args=None):
    rclpy.init(args=args)
    
    if not CAN_AVAILABLE:
        print("⚠️  注意: python-can 庫未安裝")
        print("安裝指令: pip install python-can")
        print("將以模擬模式運行，僅顯示資料內容")
        print("")
    
    node = CANSystemTestPublisher()
    
    try:
        print("🚀 CAN系統測試開始...")
        print("📊 觀察資料輸出確認格式正確")
        print("⏹️  按 Ctrl+C 停止測試")
        print("")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🔴 CAN系統測試發布器已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
