#!/usr/bin/env python3
"""
RTK GPS Reader for UBlox ZED-F9P with RTK2GO NTRIP corrections
使用 pyubx2 庫進行 UBX 消息解析
"""

import serial
import socket
import threading
import time
import base64
from pyubx2 import UBXReader, UBXMessage, POLL
import logging

# 設置日誌
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class RTKGPSReader:
    def __init__(self, serial_port='/dev/ttyUSB0', baudrate=38400):
        """
        初始化 RTK GPS 讀取器
        
        Args:
            serial_port (str): GPS 模組的串口路徑
            baudrate (int): 波特率
        """
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.serial_conn = None
        self.ntrip_socket = None
        self.running = False
        
        # NTRIP 配置 (使用你提供的參數)
        self.ntrip_config = {
            'user': "chris920325@gmail.com",
            'password': "nturt2023",
            'caster': "3.143.243.81",
            'port': 2101,
            'mountpoint': "MIE_UNIV"  # 移除前綴，在連接時動態添加
        }
        
        # GPS 數據存儲
        self.latest_position = {}
        self.rtk_status = "No Fix"
        
    def connect_serial(self):
        """連接到 GPS 模組"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=1
            )
            logger.info(f"已連接到 GPS 模組: {self.serial_port}")
            return True
        except Exception as e:
            logger.error(f"連接 GPS 模組失敗: {e}")
            return False
    
    def test_ntrip_sources(self):
        """測試 NTRIP 源列表"""
        try:
            test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            test_socket.settimeout(10)
            test_socket.connect((self.ntrip_config['caster'], self.ntrip_config['port']))
            
            # 請求源列表
            request = "GET / HTTP/1.1\r\nHost: {}\r\nUser-Agent: NTRIP SourceTable/1.0\r\n\r\n".format(
                self.ntrip_config['caster']
            )
            test_socket.send(request.encode())
            
            # 讀取響應
            response = b""
            while True:
                data = test_socket.recv(1024)
                if not data:
                    break
                response += data
                if len(response) > 10000:  # 限制響應大小
                    break
            
            response_str = response.decode('utf-8', errors='ignore')
            logger.info("可用的 NTRIP 源:")
            
            # 解析源表
            lines = response_str.split('\n')
            for line in lines:
                if line.startswith('STR;'):
                    parts = line.split(';')
                    if len(parts) > 1:
                        mountpoint = parts[1]
                        logger.info(f"  - {mountpoint}")
            
            test_socket.close()
            
        except Exception as e:
            logger.error(f"獲取 NTRIP 源列表失敗: {e}")
    
    def connect_ntrip(self):
        """連接到 NTRIP 伺服器"""
        try:
            logger.info(f"正在連接 NTRIP: {self.ntrip_config['caster']}:{self.ntrip_config['port']}")
            
            # 創建 socket 連接
            self.ntrip_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.ntrip_socket.settimeout(15)  # 15秒連接超時
            
            # 嘗試連接
            self.ntrip_socket.connect((self.ntrip_config['caster'], self.ntrip_config['port']))
            logger.info("Socket 連接成功")
            
            # 確保掛載點有正確的前綴
            mountpoint = self.ntrip_config['mountpoint']
            if not mountpoint.startswith('/'):
                mountpoint = '/' + mountpoint
            
            # 構建標準 NTRIP 請求
            auth_string = f"{self.ntrip_config['user']}:{self.ntrip_config['password']}"
            auth_bytes = base64.b64encode(auth_string.encode()).decode()
            
            request = (
                f"GET {mountpoint} HTTP/1.1\r\n"
                f"Host: {self.ntrip_config['caster']}:{self.ntrip_config['port']}\r\n"
                f"User-Agent: NTRIP PythonClient/1.0\r\n"
                f"Accept: */*\r\n"
                f"Connection: close\r\n"
                f"Authorization: Basic {auth_bytes}\r\n"
                f"\r\n"
            )
            
            logger.info(f"請求掛載點: {mountpoint}")
            logger.debug(f"發送 NTRIP 請求:\n{request}")
            self.ntrip_socket.send(request.encode())
            
            # 讀取響應頭（添加超時保護）
            response = b""
            start_time = time.time()
            while b"\r\n\r\n" not in response:
                if time.time() - start_time > 10:  # 10秒響應超時
                    raise Exception("NTRIP 響應超時")
                    
                self.ntrip_socket.settimeout(1)
                try:
                    data = self.ntrip_socket.recv(1)
                    if not data:
                        break
                    response += data
                except socket.timeout:
                    continue
            
            response_str = response.decode('utf-8', errors='ignore')
            logger.info(f"NTRIP 響應前 100 字符: {response_str[:100]}")
            
            # 檢查是否是源表響應（當掛載點不存在時會返回源表）
            if "SOURCETABLE 200 OK" in response_str:
                logger.warning(f"收到源表響應，掛載點 {mountpoint} 可能不存在")
                logger.info("正在解析可用的掛載點...")
                
                # 解析源表
                lines = response_str.split('\n')
                available_mounts = []
                for line in lines:
                    if line.startswith('STR;'):
                        parts = line.split(';')
                        if len(parts) > 1:
                            mount = parts[1]
                            available_mounts.append(mount)
                            logger.info(f"  可用掛載點: {mount}")
                
                if available_mounts:
                    logger.warning(f"建議使用以下掛載點之一: {', '.join(available_mounts[:5])}")
                
                return False
                
            elif "200 OK" in response_str or "ICY 200 OK" in response_str:
                logger.info(f"NTRIP 連接成功，掛載點: {mountpoint}")
                self.ntrip_socket.settimeout(5)  # 設置數據接收超時
                return True
            else:
                logger.error(f"NTRIP 連接失敗，響應: {response_str[:200]}")
                if self.ntrip_socket:
                    self.ntrip_socket.close()
                    self.ntrip_socket = None
                return False
                
        except socket.timeout:
            logger.error("NTRIP 連接超時")
            if self.ntrip_socket:
                self.ntrip_socket.close()
                self.ntrip_socket = None
            return False
        except Exception as e:
            logger.error(f"NTRIP 連接錯誤: {e}")
            if self.ntrip_socket:
                self.ntrip_socket.close()
                self.ntrip_socket = None
            return False
    
    def ntrip_thread(self):
        """NTRIP 數據接收線程"""
        while self.running:
            try:
                if self.ntrip_socket:
                    # 接收 RTCM 修正數據
                    rtcm_data = self.ntrip_socket.recv(1024)
                    if rtcm_data and self.serial_conn:
                        # 將 RTCM 數據發送給 GPS 模組
                        self.serial_conn.write(rtcm_data)
                        logger.debug(f"發送 RTCM 數據: {len(rtcm_data)} bytes")
                    elif not rtcm_data:
                        logger.warning("NTRIP 連接中斷")
                        break
                    
            except socket.timeout:
                # 超時是正常的，繼續循環
                continue
            except Exception as e:
                logger.error(f"NTRIP 數據接收錯誤: {e}")
                time.sleep(1)
    
    def parse_ubx_message(self, msg):
        """解析 UBX 消息"""
        try:
            if hasattr(msg, 'identity'):
                msg_class = msg.identity
                
                # 解析 NAV-PVT 消息 (位置、速度、時間)
                if msg_class == 'NAV-PVT':
                    # 檢查消息是否有所需的屬性
                    if hasattr(msg, 'lat') and hasattr(msg, 'lon'):
                        # 調試：打印原始數據
                        raw_lat = getattr(msg, 'lat', 0)
                        raw_lon = getattr(msg, 'lon', 0)
                        logger.debug(f"原始座標: lat={raw_lat}, lon={raw_lon}")
                        
                        # UBlox 座標格式是 1e-7 度 (即需要除以 10,000,000)
                        lat = raw_lat / 1e7
                        lon = raw_lon / 1e7
                        
                        # 如果座標看起來太小，可能是其他格式
                        if abs(lat) < 0.01 and abs(lon) < 0.01 and (raw_lat != 0 or raw_lon != 0):
                            logger.warning(f"座標值異常小: {lat:.8f}, {lon:.8f}")
                            logger.warning(f"原始值: {raw_lat}, {raw_lon}")
                            # 嘗試其他縮放因子
                            if abs(raw_lat) > 1000:
                                lat = raw_lat / 1e2  # 嘗試 1e-2 度
                                lon = raw_lon / 1e2
                                logger.info(f"使用 1e-2 縮放: {lat:.8f}, {lon:.8f}")
                        
                        self.latest_position = {
                            'latitude': lat,
                            'longitude': lon,
                            'altitude': getattr(msg, 'hMSL', 0) / 1000.0,  # 米
                            'speed': getattr(msg, 'gSpeed', 0) / 1000.0,  # m/s
                            'heading': getattr(msg, 'headMot', 0) / 1e5,  # 度
                            'satellites': getattr(msg, 'numSV', 0),
                            'fix_type': getattr(msg, 'fixType', 0),
                            'raw_lat': raw_lat,  # 保存原始值用於調試
                            'raw_lon': raw_lon,
                        }
                        
                        # 安全地獲取時間信息
                        if hasattr(msg, 'year') and hasattr(msg, 'month') and hasattr(msg, 'day'):
                            year = getattr(msg, 'year', 2025)
                            month = getattr(msg, 'month', 1)
                            day = getattr(msg, 'day', 1)
                            hour = getattr(msg, 'hour', 0)
                            minute = getattr(msg, 'min', 0)
                            second = getattr(msg, 'second', 0)  # 修正屬性名
                            self.latest_position['timestamp'] = f"{year}-{month:02d}-{day:02d} {hour:02d}:{minute:02d}:{second:02d}"
                        else:
                            self.latest_position['timestamp'] = "Unknown"
                        
                        # 判斷 RTK 狀態 - 修正 flags 屬性檢查
                        fix_type = getattr(msg, 'fixType', 0)
                        if fix_type == 3:  # 3D Fix
                            # 檢查是否有 carrSoln 屬性來判斷 RTK 狀態
                            if hasattr(msg, 'carrSoln'):
                                carr_soln = getattr(msg, 'carrSoln', 0)
                                if carr_soln == 2:  # RTK Fixed
                                    self.rtk_status = "RTK Fixed"
                                elif carr_soln == 1:  # RTK Float
                                    self.rtk_status = "RTK Float"
                                else:
                                    self.rtk_status = "3D Fix"
                            else:
                                self.rtk_status = "3D Fix"
                        elif fix_type == 2:  # 2D Fix
                            self.rtk_status = "2D Fix"
                        else:
                            self.rtk_status = "No Fix"
                        
                        # 只有在座標看起來合理時才記錄
                        if abs(lat) > 0.1 or abs(lon) > 0.1:
                            logger.debug(f"位置更新: {lat:.6f}, {lon:.6f}")
                
                # 解析 NAV-STATUS 消息 (狀態信息)
                elif msg_class == 'NAV-STATUS':
                    gps_fix = getattr(msg, 'gpsFix', 0)
                    flags = getattr(msg, 'flags', 0)
                    logger.debug(f"GPS 狀態: Fix={gps_fix}, Flags={flags}")
                    
                # 添加其他有用的消息類型
                elif msg_class == 'NAV-SAT':
                    # 衛星信息
                    num_svs = getattr(msg, 'numSvs', 0)
                    logger.debug(f"可見衛星數: {num_svs}")
                    
        except Exception as e:
            logger.error(f"解析 UBX 消息錯誤: {e}")
            # 打印消息屬性以便調試
            if hasattr(msg, '__dict__'):
                logger.debug(f"消息屬性: {list(msg.__dict__.keys())}")
                if hasattr(msg, 'identity'):
                    logger.debug(f"消息類型: {msg.identity}")
    
    def gps_read_thread(self):
        """GPS 數據讀取線程"""
        if not self.serial_conn:
            logger.error("串口未連接")
            return
            
        logger.info("GPS 數據讀取線程開始運行")
        ubx_reader = UBXReader(self.serial_conn)
        
        while self.running:
            try:
                # 讀取並解析 UBX 消息
                raw_data, parsed_data = ubx_reader.read()
                
                if parsed_data:
                    self.parse_ubx_message(parsed_data)
                elif raw_data:
                    # 如果有原始數據但無法解析，記錄日誌
                    logger.debug(f"收到無法解析的數據: {raw_data[:20]}")
                    
            except Exception as e:
                logger.error(f"GPS 數據讀取錯誤: {e}")
                time.sleep(0.1)
        
        logger.info("GPS 數據讀取線程結束")
    
    def configure_receiver(self):
        """配置接收器以獲得最佳 RTK 性能"""
        try:
            # 等待接收器穩定
            time.sleep(2)
            
            # 使用簡單的配置方法
            # 啟用 NAV-PVT 消息 (每秒1次)
            nav_pvt_enable = b'\xB5\x62\x06\x01\x08\x00\x01\x07\x01\x00\x00\x00\x00\x00\x18\x61'
            self.serial_conn.write(nav_pvt_enable)
            time.sleep(0.1)
            
            # 啟用 NAV-STATUS 消息
            nav_status_enable = b'\xB5\x62\x06\x01\x08\x00\x01\x03\x01\x00\x00\x00\x00\x00\x14\x59'  
            self.serial_conn.write(nav_status_enable)
            time.sleep(0.1)
            
            logger.info("接收器配置完成")
            
        except Exception as e:
            logger.error(f"配置接收器失敗: {e}")
            logger.info("將使用預設配置繼續運行")
    
    def start(self):
        """啟動 RTK GPS 讀取"""
        logger.info("啟動 RTK GPS 讀取器...")
        
        # 連接串口
        if not self.connect_serial():
            return False
        
        # 配置接收器
        self.configure_receiver()
        
        # 先啟動 GPS 讀取線程
        self.running = True
        gps_thread = threading.Thread(target=self.gps_read_thread, daemon=True)
        gps_thread.start()
        logger.info("GPS 讀取線程已啟動")
        
        # 在背景嘗試連接 NTRIP（非阻塞）
        ntrip_connect_thread = threading.Thread(target=self._connect_ntrip_background, daemon=True)
        ntrip_connect_thread.start()
        
        logger.info("RTK GPS 讀取器已啟動")
        return True
    
    def _connect_ntrip_background(self):
        """背景連接 NTRIP"""
        logger.info("正在背景連接 NTRIP...")
        if self.connect_ntrip():
            logger.info("NTRIP 連接成功，啟動 NTRIP 數據線程")
            ntrip_thread = threading.Thread(target=self.ntrip_thread, daemon=True)
            ntrip_thread.start()
        else:
            logger.warning("NTRIP 連接失敗，將以普通 GPS 模式運行")
    
    def stop(self):
        """停止 RTK GPS 讀取"""
        logger.info("正在停止 RTK GPS 讀取器...")
        self.running = False
        
        if self.serial_conn:
            self.serial_conn.close()
        
        if self.ntrip_socket:
            self.ntrip_socket.close()
        
        logger.info("RTK GPS 讀取器已停止")
    
    def get_position(self):
        """獲取當前位置信息"""
        return self.latest_position.copy() if self.latest_position else None
    
    def get_rtk_status(self):
        """獲取 RTK 狀態"""
        return self.rtk_status
    
    def print_status(self):
        """打印當前狀態"""
        if self.latest_position:
            pos = self.latest_position
            print(f"\n{'='*50}")
            print(f"RTK 狀態: {self.rtk_status}")
            print(f"時間: {pos.get('timestamp', 'N/A')}")
            print(f"緯度: {pos.get('latitude', 0):.8f}°")
            print(f"經度: {pos.get('longitude', 0):.8f}°")
            print(f"海拔: {pos.get('altitude', 0):.2f} m")
            print(f"速度: {pos.get('speed', 0):.2f} m/s")
            print(f"航向: {pos.get('heading', 0):.1f}°")
            print(f"衛星數: {pos.get('satellites', 0)}")
            print(f"定位類型: {pos.get('fix_type', 0)}")
            
            # 顯示原始座標值用於調試
            if 'raw_lat' in pos and 'raw_lon' in pos:
                print(f"原始座標: lat={pos['raw_lat']}, lon={pos['raw_lon']}")
            
            # 檢查座標是否合理
            lat, lon = pos.get('latitude', 0), pos.get('longitude', 0)
            if abs(lat) < 0.01 and abs(lon) < 0.01:
                print("⚠️  警告: 座標值異常小，可能需要檢查數據格式")
            elif 10 < abs(lat) < 90 and 10 < abs(lon) < 180:
                print("✅ 座標值看起來正常")
            
            # NTRIP 狀態
            ntrip_status = "已連接" if self.ntrip_socket else "未連接"
            print(f"NTRIP 狀態: {ntrip_status}")
            
            print(f"{'='*50}")
        else:
            print("等待 GPS 數據...")


def main():
    """主函數 - 示例用法"""
    import sys
    
    # 檢查命令行參數
    if len(sys.argv) > 1 and sys.argv[1] == '--debug':
        logging.getLogger().setLevel(logging.DEBUG)
        print("調試模式已啟用")
    
    # 創建 RTK GPS 讀取器實例
    # 請根據你的系統修改串口路徑
    # Linux: /dev/ttyUSB0, /dev/ttyACM0
    # Windows: COM3, COM4, etc.
    gps_reader = RTKGPSReader(serial_port='/dev/ttyACM0', baudrate=38400)  # 使用你的串口
    
    # 可選：測試 NTRIP 源
    if len(sys.argv) > 1 and sys.argv[1] == '--sources':
        print("測試 NTRIP 源列表...")
        gps_reader.test_ntrip_sources()
        return
    
    # 可選：僅測試 NTRIP 連接
    if len(sys.argv) > 1 and sys.argv[1] == '--test-ntrip':
        print("測試 NTRIP 連接...")
        if gps_reader.connect_ntrip():
            print("NTRIP 連接成功！")
        else:
            print("NTRIP 連接失敗！")
        return
    
    try:
        # 啟動讀取器
        if gps_reader.start():
            print("RTK GPS 讀取器已啟動，按 Ctrl+C 停止...")
            print("如果 NTRIP 連接失敗，請檢查:")
            print("1. 網路連接")
            print("2. 帳號密碼是否正確")
            print("3. 掛載點是否存在")
            print("使用 --sources 參數查看可用的掛載點")
            print("使用 --test-ntrip 參數單獨測試 NTRIP 連接")
            print("-" * 50)
            
            # 等待幾秒讓 GPS 開始接收數據
            time.sleep(3)
            
            # 主循環 - 每秒顯示一次狀態
            while True:
                time.sleep(1)
                gps_reader.print_status()
                
    except KeyboardInterrupt:
        print("\n接收到停止信號...")
    finally:
        gps_reader.stop()


if __name__ == "__main__":
    main()