import time
import threading
import queue
import signal

import numpy as np
from ekf import EKF2D
from pipeline import sensor_simulator, STOP, ImuMsg, GpsMsg
from tcp_pub import TcpOdomPublisher

def ekf_worker(
    imu_q: queue.Queue,
    gps_q: queue.Queue,
    stop_evt: threading.Event,
    pub: TcpOdomPublisher,
    print_hz: float = 1.0,
):
    ekf = EKF2D()

    last_ts = None
    last_print = time.time()
    gps_stopped = False
    imu_stopped = False

    while not stop_evt.is_set():
        # We want to process IMU frequently; GPS when available.
        # Non-blocking-ish reads with timeouts:
        try:
            imu_item = imu_q.get(timeout=0.2)
        except queue.Empty:
            imu_item = None

        if imu_item is STOP:
            imu_stopped = True
            imu_item = None

        if imu_item is not None:
            assert isinstance(imu_item, ImuMsg)
            if last_ts is None:
                last_ts = imu_item.ts
                continue
            dt = max(1e-3, imu_item.ts - last_ts)
            last_ts = imu_item.ts

            ekf.predict(a=np.array([imu_item.ax, imu_item.ay]), dt=dt)

        # GPS updates (can be multiple queued)
        while True:
            try:
                gps_item = gps_q.get_nowait()
            except queue.Empty:
                break

            if gps_item is STOP:
                gps_stopped = True
                break

            assert isinstance(gps_item, GpsMsg)
            ekf.update_gps(z=np.array([gps_item.px, gps_item.py]))

        # publish latest
        px, py, vx, vy = ekf.x.tolist()
        pub.update_state(ts=time.time(), px=px, py=py, vx=vx, vy=vy)

        # print status
        now = time.time()
        if now - last_print >= 1.0 / print_hz:
            last_print = now
            print(f"[ekf] px={px:7.2f} py={py:7.2f} vx={vx:6.2f} vy={vy:6.2f}")

        if imu_stopped and gps_stopped:
            break

def main():
    stop_evt = threading.Event()

    # graceful Ctrl+C
    def _sigint(sig, frame):
        stop_evt.set()
    signal.signal(signal.SIGINT, _sigint)

    imu_q = queue.Queue(maxsize=500)
    gps_q = queue.Queue(maxsize=200)

    # TCP publisher
    pub = TcpOdomPublisher(host="0.0.0.0", port=9000, send_hz=10.0)
    pub.start()
    print("[main] TCP odom publisher on port 9000 (JSON lines)")

    # threads
    t_sensor = threading.Thread(
        target=sensor_simulator,
        args=(imu_q, gps_q, stop_evt),
        kwargs={"imu_hz": 50.0, "gps_hz": 5.0},
        daemon=True,
    )
    t_ekf = threading.Thread(
        target=ekf_worker,
        args=(imu_q, gps_q, stop_evt, pub),
        daemon=True,
    )

    t_sensor.start()
    t_ekf.start()

    print("[main] Running... Press Ctrl+C to stop.")
    while not stop_evt.is_set():
        time.sleep(0.2)

    print("[main] Stopping...")
    pub.stop()
    t_sensor.join(timeout=2.0)
    t_ekf.join(timeout=2.0)
    print("[main] Stopped.")

if __name__ == "__main__":
    main()