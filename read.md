# Python Robotics Stack Demo

A multithreaded robotics-style data processing pipeline implemented in Python.

This project demonstrates:

- Multithreaded sensor simulation (IMU + GPS)
- Extended Kalman Filter (EKF) for state estimation
- Real-time data fusion
- TCP-based odometry broadcasting (JSON protocol)
- Embedded/robot-inspired software architecture

---

## System Architecture

```
Sensor Thread
   ├── IMU (50 Hz)
   ├── GPS (5 Hz)
        ↓
EKF Worker Thread
   ├── Prediction (IMU)
   ├── Update (GPS)
        ↓
TCP Publisher Thread
   ├── Multi-client broadcast
   ├── JSON line protocol
```

This design mimics real robotics middleware systems:

- Producer → Processing → Publisher
- Thread-safe queues
- Bounded buffers
- Graceful shutdown

---

## Features

- 2D Extended Kalman Filter (state: px, py, vx, vy)
- Separate prediction and update steps
- Covariance propagation
- Multi-client TCP streaming
- JSON line-delimited protocol
- Clean Ctrl+C shutdown handling

---

## Tech Stack

- Python 3.10+
- threading
- queue
- numpy
- socket

---

## Project Structure

```
robot_stack_demo/
│
├── app.py              # Entry point
├── ekf.py              # EKF implementation
├── pipeline.py         # Sensor simulator (IMU + GPS)
├── tcp_pub.py          # TCP odometry broadcaster
├── client_test.py      # TCP client demo
├── requirements.txt
└── README.md
```

---

## Installation (Windows)

```bash
git clone https://github.com/yitingHH-bit/python_projects.git
cd python_projects

python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

---

## Run the System

### Terminal 1

```bash
python app.py
```

You will see EKF pose estimation output like:

```
[ekf] px=  12.34 py=  -5.67 vx= 1.23 vy= 0.98
```

### Terminal 2 (Optional - TCP Client)

```bash
python client_test.py
```

Example output:

```json
{"ts": 1720000000.123, "px": 10.5, "py": -2.3, "vx": 1.1, "vy": 0.9}
```

---

## EKF Model

State vector:

```
x = [px, py, vx, vy]
```

Prediction step:

```
x_k = f(x_{k-1}, a_k)
P_k = F P F^T + Q
```

Update step (GPS):

```
K = P H^T (H P H^T + R)^-1
x = x + K(y)
P = (I - K H)P
```

IMU provides high-frequency prediction.
GPS provides lower-frequency correction.

---

## Communication Protocol

JSON newline-delimited format:

```json
{
  "ts": 123.456,
  "px": 1.0,
  "py": 2.0,
  "vx": 0.1,
  "vy": 0.2
}
```

This protocol allows:

- Easy debugging
- Cross-language compatibility
- Integration with external systems

---

## Robotics Relevance

This project demonstrates concepts commonly used in:

- Mobile robotics
- Autonomous vehicles
- Robot localization
- ROS-like middleware systems
- Embedded real-time systems
- Multi-sensor data fusion

---

## Possible Extensions

- Add orientation (yaw, quaternion)
- Publish covariance matrix
- Integrate LiDAR
- Switch to UDP/DDS-style communication
- Add logging module
- Containerize with Docker

---

## Engineering Highlights

- Thread-safe shared state
- Bounded queue for backpressure
- Separation of concerns
- Modular architecture
- Scalable publisher design

---

## License

For educational and demonstration purposes.
