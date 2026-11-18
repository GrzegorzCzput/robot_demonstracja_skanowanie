# Runtime for follow_trajectory.py. Built automatically by run_robot_docker.sh.
FROM python:3.13-slim

RUN apt-get update && apt-get install -y --no-install-recommends git \
    && rm -rf /var/lib/apt/lists/*

# What the official TRLC-DK1 driver needs (mujoco is used for gravity compensation).
RUN pip install --no-cache-dir numpy==2.5.2 pyserial==3.5 mujoco==4.0.1

# Official TRLC-DK1 driver and robot model, pinned to a tested commit.
RUN git clone https://github.com/robot-learning-co/trlc-dk1.git /opt/trlc-dk1 \
    && git -C /opt/trlc-dk1 checkout 12f5368aefd0381461f2c7ffbb5611b4e8c90de9
ENV PYTHONPATH=/opt/trlc-dk1 PYTHONUNBUFFERED=1

# run_robot_docker.sh mounts this folder at /app.
WORKDIR /app
ENTRYPOINT ["python", "follow_trajectory.py"]
