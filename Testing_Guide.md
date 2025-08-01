# 🚗 GPS CAN系統測試指南

## 概述
這個測試套件提供完整的GPS和CAN系統測試功能，讓您可以測試整個資料流程：
**ROS2資料 → CAN編碼 → CAN解碼 → 日誌輸出**

## 🛠️ 安裝和構建

```bash
# 1. 進入工作空間
cd /home/docker/ws

# 2. 構建包
colcon build --packages-select gps_tracker

# 3. 載入環境變數
source install/setup.bash
```

## 🧪 測試模式

### 1. 完整系統測試（推薦）

**目標**: 測試GPS編碼解碼完整流程

```bash
# 終端1: 啟動ROS2資料發布器
ros2 run gps_tracker system_test_pub all

# 終端2: 啟動CAN編碼器
ros2 run gps_tracker gps_can_pub_newEncode

# 終端3: 啟動CAN解碼器
ros2 run gps_tracker gps_can_rec
```

**預期結果**:
- 終端1顯示模擬的GPS和速度資料
- 終端2顯示CAN編碼輸出
- 終端3顯示CAN解碼結果
- 數值應該一致（考慮編碼精度損失）

### 2. 高頻率性能測試

**目標**: 測試系統在高負載下的表現

```bash
# 終端1: 啟動高頻資料發布器
ros2 run gps_tracker system_test_pub high

# 終端2-3: 同上啟動編碼器和解碼器
```

**特點**:
- GPS: 20Hz 更新頻率
- 速度: 100Hz 更新頻率
- 測試系統性能極限

### 3. 靜態基礎測試

**目標**: 驗證基本功能正確性

```bash
# 終端1: 啟動靜態資料發布器
ros2 run gps_tracker system_test_pub static

# 終端2-3: 同上啟動編碼器和解碼器
```

**特點**:
- 固定GPS位置和速度
- 便於驗證編碼解碼準確性
- 適合初次測試

### 4. CAN系統測試

**目標**: 測試蓄電池和逆變器CAN協議

```bash
# 啟動CAN系統測試發布器
ros2 run gps_tracker can_test_pub

# 在另一個終端啟動解碼器
ros2 run gps_tracker gps_can_rec
```

**包含的系統**:
- 🔋 蓄電池系統 (電壓、溫度、SOC、狀態)
- ⚡ 逆變器系統 (四輪狀態、電源、溫度)
- ⏰ 時間戳同步
- 💓 心跳信號

## 📊 資料驗證

### GPS資料驗證
檢查以下數值的一致性：

| 資料類型 | 發布器輸出 | 解碼器輸出 | 容許誤差 |
|---------|-----------|-----------|----------|
| 緯度 | 25.013xxx | 25.013xxx | ±0.0001° |
| 經度 | 121.528xxx | 121.528xxx | ±0.0001° |
| 高度 | xx.x m | xx.x m | ±1m |
| 速度 | xx.x km/h | xx.x km/h | ±1km/h |

### CAN資料驗證
檢查CAN解碼輸出：

```bash
# 蓄電池系統應顯示：
[ACCUMULATOR] Cell Voltages (Index X): [3.x, 3.x, ...]V
[ACCUMULATOR] Temperatures (Index X): [xx.x, xx.x, ...]°C  
[ACCUMULATOR] SOC: xx%, Current: xx.xxA, Capacity: xx.xxAh
[ACCUMULATOR] Heartbeat: OK

# 逆變器系統應顯示：
[INVERTER FL] Status: 0xxxxx, Torque: x.xxx, Speed: xxxxRPM
[INVERTER FL] DC Voltage: xxx.xxV, DC Current: xx.xxA
[INVERTER FL] Temps - MOS: xx.x°C, MCU: xx.x°C, Motor: xx.x°C
[INVERTER FL] Heartbeat: OK
```

## 🔧 故障排除

### 常見問題

#### 1. CAN介面錯誤
```
錯誤: [Errno 19] No such device
```
**解決方案**:
```bash
# 檢查CAN介面
ip link show can0

# 如果不存在，創建虛擬CAN介面
sudo modprobe vcan
sudo ip link add dev can0 type vcan
sudo ip link set up can0
```

#### 2. 沒有資料輸出
**檢查清單**:
- [ ] 所有節點都在運行
- [ ] ROS2環境變數已載入
- [ ] CAN介面已啟動
- [ ] 沒有防火牆阻攔

#### 3. 資料不一致
**可能原因**:
- 編碼精度限制（正常現象）
- 時間延遲
- 數值溢出

#### 4. Python CAN庫問題
```
ImportError: No module named 'can'
```
**解決方案**:
```bash
pip install python-can
```

## 📈 效能監控

### 監控CPU使用率
```bash
# 監控ROS2節點CPU使用率
top -p $(pgrep -f "ros2")
```

### 監控CAN匯流排負載
```bash
# 安裝can-utils
sudo apt install can-utils

# 監控CAN訊息
candump can0

# 統計CAN流量
canstat can0
```

### 檢查ROS2話題頻率
```bash
# 檢查GPS資料頻率
ros2 topic hz /fix

# 檢查速度資料頻率  
ros2 topic hz /vel
```

## 🎯 測試場景

### 場景1: 基本功能驗證
1. 啟動靜態測試
2. 確認所有資料正常輸出
3. 驗證數值準確性

### 場景2: 長時間穩定性測試
1. 啟動完整系統測試
2. 運行30分鐘以上
3. 監控記憶體洩漏和錯誤

### 場景3: 極限效能測試
1. 啟動高頻測試
2. 監控CPU和記憶體使用率
3. 檢查資料遺失情況

### 場景4: 錯誤恢復測試
1. 在運行中停止某個節點
2. 重新啟動節點
3. 確認系統恢復正常

## 📝 測試報告範本

```
測試日期: ____
測試者: ____
測試環境: Ubuntu __ / ROS2 __

✅ 基本功能測試
- [ ] GPS資料正確編碼/解碼
- [ ] 速度資料正確編碼/解碼  
- [ ] CAN系統資料正常
- [ ] 心跳信號正常

⚡ 效能測試
- CPU使用率: __%
- 記憶體使用量: __MB
- CAN匯流排負載: __%
- 資料遺失率: __%

🔧 問題記錄
問題1: ____
解決方案: ____

問題2: ____
解決方案: ____

總結: ____
```

## 🚀 下一步

測試完成後，您可以：

1. **整合到實際系統**: 將測試好的編碼器部署到實際車輛
2. **開發Web儀表板**: 使用已創建的React儀表板架構
3. **添加更多感測器**: 擴展CAN協議支援更多設備
4. **實時監控**: 部署到Raspberry Pi進行即時監控

---

💡 **提示**: 建議先從靜態測試開始，確認基本功能正常後再進行高頻測試。
