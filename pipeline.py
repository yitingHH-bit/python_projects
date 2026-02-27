import time
import threading
import queue
import numpy as np
from dataclasses import dataclass

@dataclass
class ImuMsg:
    ts: float
    ax: float
    ay: float

@dataclass
class GpsMsg:
    ts: float
    px: float
    py: float

STOP = object()

def _true_acc(t: float) -> np.ndarray:
    # A smooth curve acceleration (demo)
    return np.array([0.3*np.cos(0.4*t), 0.3*np.sin(0.4*t)], dtype=float)

def sensor_simulator(
    imu_q: queue.Queue,
    gps_q: queue.Queue,
    stop_evt: threading.Event,
    imu_hz: float = 50.0,
    gps_hz: float = 5.0,
    imu_noise: float = 0.15,
    gps_noise: float = 1.2,
):
    """
    Simulate "truth" motion internally, publish IMU accel at imu_hz,
    publish GPS position at gps_hz.
    """
    rng = np.random.default_rng(0)

    dt = 1.0 / imu_hz
    gps_period = 1.0 / gps_hz
    next_gps = 0.0

    # truth state: [px, py, vx, vy]
    x_true = np.array([0.0, 0.0, 2.0, 0.0], dtype=float)

    t0 = time.time()
    sim_t = 0.0

    while not stop_evt.is_set():
        a = _true_acc(sim_t)

        # propagate truth
        px, py, vx, vy = x_true
        px = px + vx*dt + 0.5*a[0]*dt*dt
        py = py + vy*dt + 0.5*a[1]*dt*dt
        vx = vx + a[0]*dt
        vy = vy + a[1]*dt
        x_true = np.array([px, py, vx, vy], dtype=float)

        now = time.time()

        # IMU
        a_meas = a + rng.normal(0.0, imu_noise, size=2)
        imu_msg = ImuMsg(ts=now, ax=float(a_meas[0]), ay=float(a_meas[1]))
        try:
            imu_q.put(imu_msg, timeout=0.01)
        except queue.Full:
            pass

        # GPS (lower rate)
        if sim_t + 1e-9 >= next_gps:
            z = x_true[:2] + rng.normal(0.0, gps_noise, size=2)
            gps_msg = GpsMsg(ts=now, px=float(z[0]), py=float(z[1]))
            try:
                gps_q.put(gps_msg, timeout=0.01)
            except queue.Full:
                pass
            next_gps += gps_period

        sim_t += dt

        # keep real-time-ish
        elapsed = time.time() - t0
        if sim_t > elapsed:
            time.sleep(sim_t - elapsed)

    # stop markers
    imu_q.put(STOP)
    gps_q.put(STOP)