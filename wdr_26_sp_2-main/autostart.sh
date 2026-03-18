sleep 20
cd /home/wdr/wdr_26_sp_3/wdr_26_sp_2-main/
screen \
    # -L \
    # -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    # -d \
    # -m \
    bash -c "pkill -9 rw_tracker_test"
    bash -c "./build/rw_tracker_test"
