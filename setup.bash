source /opt/ros/lyrical/setup.bash
if [[ -f ~/ros/install/setup.bash ]]; then source ~/ros/install/setup.bash; fi

export ROS_DOMAIN_ID=26
export ROS_NAMESPACE="/klemen"

ros2() {
    if [[ "$1" == "run" && -n "$ROS_NAMESPACE" && "$ROS_NAMESPACE" != "/" ]]; then
        local arg
        for arg in "$@"; do
            if [[ "$arg" == *__ns:=* ]]; then
                command ros2 "$@"
                return
            fi
        done
        command ros2 "$@" --ros-args -r "__ns:=$ROS_NAMESPACE"
    else
        command ros2 "$@"
    fi
}
#export ROS2_EASY_MODE=10.8.134.137
# Connect this robot's ROS nodes to the lab Zenoh router (otherwise the PC can't see them)
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/10.8.134.137:7447"]'
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
