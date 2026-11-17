# Minimal runtime for follow_trajectory.py on a real TRLC-DK1 (no GPU, no cuRobo).
FROM python:3.13-slim

RUN apt-get update && apt-get install -y --no-install-recommends git \
    && rm -rf /var/lib/apt/lists/*

# Only what the official driver (trlc_dk1_control + DM_CAN) actually imports.
RUN pip install --no-cache-dir numpy==2.5.2 pyserial==3.5 mujoco==4.0.1

# Full upstream repo: the vendored third_party copy lacks the DM_CAN motor library.
RUN git clone https://github.com/robot-learning-co/trlc-dk1.git /opt/trlc-dk1 \
    && git -C /opt/trlc-dk1 checkout 12f5368aefd0381461f2c7ffbb5611b4e8c90de9
ENV PYTHONPATH=/opt/trlc-dk1 PYTHONUNBUFFERED=1

# The project is mounted read-only at /app by run_robot.sh, so edits and
# re-planned trajectories are used without rebuilding.
WORKDIR /app
ENTRYPOINT ["python", "follow_trajectory.py"]
