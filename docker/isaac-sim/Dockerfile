FROM nvcr.io/nvidia/isaac-sim:5.0.0

# Enable Isaac Sim's internal ROS2 Humble bridge with FastDDS
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DISTRO=humble
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib"

# Enable WebRTC live streaming
ENV LIVESTREAM=1
ENV PUBLIC_IP=${PUBLIC_IP:-127.0.0.1}

# Expose WebRTC ports: HTTP client, signaling, media
EXPOSE 8211 49100 47998/udp

# Copy the startup script into the container
COPY startup.py /app/startup.py

# Run the startup script via Isaac Sim's Python interpreter
ENTRYPOINT ["./python.sh", "/app/startup.py"]
