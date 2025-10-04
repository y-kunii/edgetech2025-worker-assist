# Dockerfile
FROM ros:humble

WORKDIR /workspace

# 必要パッケージ（rosbridge_server 含む）
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rosbridge-server \
 && rm -rf /var/lib/apt/lists/*

# エントリポイント
COPY rosbridge_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# 既定は「両方起動モード」
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["start"]
