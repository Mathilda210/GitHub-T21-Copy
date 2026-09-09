import os
import subprocess
import time


def run_cmd(cmd, wait=True):
    print(f"正在运行: {cmd}")

    p = subprocess.Popen(cmd, shell=True)

    if wait:
        p.wait()

    return p


def main():
    # 将 source 环境变量和启动 launch 文件合并为一条命令
    # 使用 && 连接，确保 source 成功后再执行 launch
    launch_cmd = (
        "source install_release/setup.bash && "
        "ros2 launch rslidar_sdk start.launch.py"
    )

    print("启动雷达驱动 (含刷新环境变量)...")

    lidar_proc = subprocess.Popen(
        launch_cmd,
        shell=True,
        executable="/bin/bash"  # 指定 bash 执行，因为 source 是 bash 内置命令
    )

    time.sleep(5)  # 等待驱动启动

    # 启动监测节点
    print("启动监测节点...")

    # 如果监测节点也依赖自定义工作空间的环境变量，建议也加上 source
    # 如果只是普通的 python 脚本且不依赖 ROS 自定义消息，可以不加
    monitor_cmd = (
        "source install_release/setup.bash && "
        "python3 monitor_lidar.py"
    )

    monitor_proc = subprocess.Popen(
        monitor_cmd,
        shell=True,
        executable="/bin/bash"
    )

    print("\n请手动启动rviz并检查点云显示。")
    print("按 Ctrl+C 结束所有进程。")

    try:
        lidar_proc.wait()
        monitor_proc.wait()

    except KeyboardInterrupt:
        print("\n正在关闭进程...")

        lidar_proc.terminate()
        monitor_proc.terminate()


if __name__ == "__main__":
    main()