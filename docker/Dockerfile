# Optional GUI layer. Headless calibration builds/runs in memo unchanged.
ARG BASE_IMAGE=memo_ws:latest
FROM ${BASE_IMAGE}
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rviz2 && rm -rf /var/lib/apt/lists/*
USER ros
