#!/bin/bash
# End-to-end smoke test: headless Gazebo + TurtleBot3 + Nav2 (D* Lite planner + TD3 controller).
# Sends one NavigateToPose goal in the chosen environment and reports PASS if Nav2 reports SUCCEEDED.
#
# Usage:  scripts/smoke_test.sh [environment [goal_x goal_y goal_yaw_z goal_yaw_w]]
#   environment: static_simple | static_complex (default) | dynamic_simple | dynamic_complex
# Prerequisite: the workspace is built and sourced (ROS 2 Humble). Stop any running simulation first.

ENVIRONMENT=${1:-static_complex}
case "$ENVIRONMENT" in
  static_simple)   DEFAULT_GOAL="1.6 -1.6 0.0 1.0" ;;
  static_complex)  DEFAULT_GOAL="1.5 -0.5 0.7071 0.7071" ;;
  dynamic_simple)  DEFAULT_GOAL="1.7 1.1 0.0 1.0" ;;
  dynamic_complex) DEFAULT_GOAL="3.5 -0.5 0.0 1.0" ;;
  *) echo "Unknown environment '$ENVIRONMENT'"; exit 2 ;;
esac
shift
if [ $# -ge 4 ]; then GOAL="$1 $2 $3 $4"; else GOAL="$DEFAULT_GOAL"; fi
read -r GX GY GZ GW <<< "$GOAL"

if pgrep -x gzserver > /dev/null || ros2 node list --no-daemon 2>/dev/null | grep -q "/bt_navigator"; then
  echo "A Gazebo or Nav2 simulation is already running. Stop it before running the smoke test."
  exit 2
fi

LOG=$(mktemp -d)

# Run the whole launch in its own process group so that everything it starts can be stopped.
setsid ros2 launch hybrid_navigation hybrid_simulation_launch.py \
  environment:="$ENVIRONMENT" headless:=True use_rviz:=False > "$LOG/launch.log" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT -- "-$LAUNCH_PID" 2>/dev/null
  for _ in $(seq 1 15); do
    kill -0 -- "-$LAUNCH_PID" 2>/dev/null || return
    sleep 1
  done
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
}
trap cleanup EXIT

echo "Waiting for Nav2 to become active ($ENVIRONMENT)..."
READY=0
for _ in $(seq 1 90); do
  if ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "active"; then READY=1; break; fi
  sleep 2
done
if [ "$READY" -ne 1 ]; then
  echo "FAIL: Nav2 did not become active within 180 s (logs in $LOG)"
  exit 1
fi
sleep 5   # AMCL starts at the spawn pose by itself (set from the launch file)

echo "Sending goal ($GX, $GY)..."
timeout 300 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY}, orientation: {z: $GZ, w: $GW}}}}" \
  > "$LOG/goal.log" 2>&1

if grep -q "SUCCEEDED" "$LOG/goal.log"; then
  echo "PASS: goal reached in $ENVIRONMENT (logs in $LOG)"
  exit 0
fi
echo "FAIL: goal not reached in $ENVIRONMENT (logs in $LOG)"
exit 1
