#!/usr/bin/env bash

source install/setup.bash

mode="${1:-gpu}"
if [[ $# -gt 0 ]]; then
    shift
fi

case "$mode" in
    gpu)
        ros2 launch nubot_gazebo load_world_gpu.launch.py "$@"
        ;;
    cpu)
        ros2 launch nubot_gazebo load_world_cpu.launch.py "$@"
        ;;
    *)
        echo "用法: source gameReady.sh [gpu|cpu]"
        echo "  gpu: 默认，使用 Ogre2"
        echo "  cpu: GPU 渲染失败时使用 Ogre 1.x 兼容模式"
        return 2 2>/dev/null || exit 2
        ;;
esac
