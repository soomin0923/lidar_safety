#!/bin/bash
# 실험마다 Gazebo를 완전히 새로 띄운다. 이전 프로세스가 남아 있으면 로봇이 안 뜨거나 응답이 멈춘다
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger PYTHONUNBUFFERED=1
L=~/ros2_ws/eval_all.log; : > $L

kill_all() {
  pkill -9 -f "[l]ib/lidar_safety/"; pkill -9 -f "[r]os2 launch"; pkill -9 -f "[g]zserver"
  pkill -9 -f "[g]zclient"; pkill -9 -f "[r]obot_state_publisher"; pkill -9 -f "[s]pawn_entity"
  for i in $(seq 1 20); do pgrep -f "[g]zserver|[r]os2 launch" >/dev/null || break; sleep 1; done
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
}
start_gazebo() {
  for attempt in 1 2 3; do
    kill_all
    setsid nohup ros2 launch turtlebot3_gazebo empty_world.launch.py > /tmp/empty_world.log 2>&1 &
    # 로봇이 실제로 생겨서 /odom이 나와야 준비된 것으로 본다
    for i in $(seq 1 45); do
      sleep 2
      timeout 3 ros2 topic echo /odom --once --field pose.pose.position.x >/dev/null 2>&1 && { sleep 3; echo "gazebo ready (attempt $attempt)" >> $L; return 0; }
    done
    echo "gazebo start failed (attempt $attempt)" >> $L
  done
  return 1
}
run() {
  name=$1; shift
  echo "=== $name" >> $L
  start_gazebo || { echo "=== $name SKIPPED: gazebo did not start" >> $L; return; }
  timeout 2400 ros2 run lidar_safety evaluate --name "$name" "$@" >> $L 2>&1 || echo "=== $name FAILED ($?)" >> $L
}
run main
run partial --faults zero:0.1,zero:0.3,zero:0.5,zero:0.8 --speeds 0.22
run angle30 --wall-angle 30 --speeds 0.22
run graded  --faults none,noise --monitor true,graded
kill_all
echo "=== done" >> $L
