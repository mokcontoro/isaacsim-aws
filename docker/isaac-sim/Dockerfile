FROM nvcr.io/nvidia/isaac-sim:4.2.0

# Enable Isaac Sim's internal ROS2 Humble bridge with FastDDS
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/isaac-sim/exts/omni.isaac.ros2_bridge/humble/lib"

# Copy the startup script into the container
COPY startup.py /app/startup.py

# Run the startup script via Isaac Sim's Python interpreter
ENTRYPOINT ["./python.sh", "/app/startup.py"]
