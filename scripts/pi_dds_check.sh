#!/usr/bin/env bash
set -eo pipefail

script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${script_root}/env.sh"

echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default>}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<default>}"
echo
echo "ROS nodes:"
ros2 node list || true
echo
echo "PX4 output topics:"
ros2 topic list -t | rg '^/fmu/out/(vehicle_local_position|vehicle_attitude|vehicle_status)' || true
echo
echo "One sample from VehicleLocalPosition:"
timeout 8s ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once || true
echo
echo "One sample from VehicleAttitude:"
timeout 8s ros2 topic echo /fmu/out/vehicle_attitude --qos-reliability best_effort --once || true
