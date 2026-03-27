# sleep 20
# cd /home/wdr/wdr_26_sp_3/wdr_26_sp_2-main/
# screen \
#     # -L \
#     # -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
#     # -d \
#     # -m \
#     bash -c "pkill -9 rw_tracker_test"
#     bash -c "./build/rw_tracker_test"


#!/bin/bash

# 延迟启动，确保系统初始化完成
sleep 15

cd ~/home/aaa/wdr_26_sp_2-main/ || exit 1

PROG_NAME="auto_aim_test"

# 确保日志目录存在
mkdir -p logs

while true; do
    timestamp=$(date "+%Y-%m-%d_%H-%M-%S")
    log="logs/${timestamp}.log"

    echo "[$timestamp] 清理旧进程..."

    # 杀死旧进程
    pkill -f "./build/$PROG_NAME" 2>/dev/null
    sleep 1

    echo "[$timestamp] 启动 $PROG_NAME"

    ./build/$PROG_NAME >> "$log" 2>&1 &

    pid=$!

    # 等待程序结束
    wait $pid

    echo "[$timestamp] 程序退出，5秒后重启..."

    sleep 5
done