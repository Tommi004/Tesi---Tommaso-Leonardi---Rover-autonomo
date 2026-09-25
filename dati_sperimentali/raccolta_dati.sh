#!/bin/bash
N_RUN=5

for i in $(seq 1 $N_RUN); do
  echo "=== RUN $i/$N_RUN ==="

  pkill -9 -f ros2
  pkill -9 -f gz
  sleep 10

  cd ~/rover_ws && source install/setup.bash

  nohup ros2 launch descrizione_rover simulazione.launch.py > /tmp/sim_run_$i.log 2>&1 &
  SIM_PID=$!
  sleep 25

  nohup ros2 launch configurazione_nav2 nav2.launch.py > /tmp/nav2_run_$i.log 2>&1 &
  NAV2_PID=$!
  sleep 20

  ros2 run nodo_missione navigatore_missione

  echo "Run $i completato."
  kill $SIM_PID $NAV2_PID 2>/dev/null
  pkill -9 -f ros2
  pkill -9 -f gz
  sleep 10
done

echo "Tutte le $N_RUN ripetizioni completate."
