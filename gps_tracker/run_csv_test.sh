#!/bin/bash
# 快速測試 CSV Dashboard 腳本

echo "啟動 CAN CSV Dashboard 測試..."
echo "使用 can_file.csv 以 10 倍速度播放"
echo "按 Ctrl+C 停止測試"
echo ""

cd /home/docker/ws/src/nturt_ros/gps_tracker/gps_tracker/
python3 gps_can_rec_Newest.py --csv --dashboard --speed 10.0
