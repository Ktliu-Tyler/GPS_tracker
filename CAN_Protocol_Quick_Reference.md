# CAN Protocol Quick Reference

## 快速參考表

| CAN ID | 系統 | 訊號名稱 | 用途 | 更新頻率 | 資料來源 |
|--------|------|---------|------|----------|----------|
| 0x100 | 時間同步 | Timestamp | 系統時間同步 | 1Hz | 系統 |
| 0x400 | GPS | GPS Basic | 座標資料 | 10Hz | NavSatFix |
| 0x401 | GPS | GPS Extended | 速度高度狀態 | 10Hz | NavSatFix + TwistStamped |
| 0x402 | GPS | Heading Angular | 航向和角速度 | 50Hz | TwistStamped |
| 0x403 | GPS | Acceleration | Z角速度和加速度 | 50Hz | TwistStamped |
| 0x404-0x40A | GPS擴展 | Velocity Details | 詳細速度角速度 | 50Hz | TwistStamped |
| 0x190 | 蓄電池 | Cell Voltage | 電池電壓 | 5Hz | 蓄電池管理系統 |
| 0x290 | 蓄電池 | Status | 蓄電池狀態 | 1Hz | 蓄電池管理系統 |
| 0x390 | 蓄電池 | Temperature | 溫度監測 | 1Hz | 蓄電池管理系統 |
| 0x490 | 蓄電池 | State | SOC電流容量 | 1Hz | 蓄電池管理系統 |
| 0x710 | 蓄電池 | Heartbeat | 心跳信號 | 1Hz | 蓄電池管理系統 |
| 0x191-0x194 | 逆變器 | Status | 逆變器狀態 | 10Hz | 馬達控制器 |
| 0x291-0x294 | 逆變器 | State | 電源狀態 | 10Hz | 馬達控制器 |
| 0x391-0x394 | 逆變器 | Temperature | 溫度監測 | 1Hz | 馬達控制器 |
| 0x711-0x714 | 逆變器 | Heartbeat | 心跳信號 | 1Hz | 馬達控制器 |

## 系統概覽

### 📍 GPS系統 (0x400-0x40A)
- **Ecumaster兼容格式**
- 座標、速度、航向、角速度
- 資料來源：NavSatFix + TwistStamped
- 注意：航向和加速度為預設值（無IMU感測器）

### 🔋 蓄電池系統 (0x190, 0x290, 0x390, 0x490, 0x710)
- 電池電壓監測
- 溫度監控
- SOC和電流監測
- 系統狀態和心跳

### ⚡ 逆變器系統 (0x191-0x194, 0x291-0x294, 0x391-0x394, 0x711-0x714)
- 四輪獨立控制（FL/FR/RL/RR）
- 扭矩、轉速監測
- 電源狀態監控
- 溫度監測和心跳

## 數據格式說明

### 通用規則
- **字節序**: Little-endian
- **CAN格式**: 標準11位ID
- **最大載荷**: 8字節
- **錯誤處理**: 完整異常處理機制

### 精度等級
- **標準精度**: Ecumaster兼容格式
- **高精度**: 擴展格式（0x404-0x40A）
- **監測級**: 系統狀態和心跳

## 實際應用注意事項

### 當前限制
✅ **可用資料**:
- GPS座標和高度
- 線性和角速度
- 蓄電池完整監測
- 逆變器完整監測

❌ **缺失感測器**:
- IMU加速度計
- 航向感測器
- 額外溫度感測器

### 部署建議
1. **即時監測**: 優先處理0x400-0x403 GPS資料
2. **安全監控**: 監測0x710和0x711-0x714心跳信號
3. **效能監測**: 關注逆變器狀態和蓄電池SOC
4. **除錯模式**: 使用詳細速度資料（0x404-0x40A）

---
**文檔版本**: v1.0 (2025-07-17)  
**編碼器**: gps_can_pub_newEncode.py  
**解碼器**: gps_can_rec.py  
**兼容性**: Ecumaster GPS + TS CAN Protocol
