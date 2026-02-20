FROM nvcr.io/nvidia/isaac-sim:4.2.0

# Copy the startup script into the container
COPY startup.py /app/startup.py

# Run the startup script via Isaac Sim's Python interpreter
ENTRYPOINT ["./python.sh", "/app/startup.py"]
