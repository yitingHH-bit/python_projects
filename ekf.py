import numpy as np

class EKF2D:
    """
    State x = [px, py, vx, vy]
    Control u = [ax, ay] (IMU acceleration in world frame, simplified)
    Measurement z = [px, py] (GPS position in local ENU-like frame, simplified)
    """
    def __init__(self, x0=None, P0=None, Q=None, R=None):
        self.x = np.zeros(4) if x0 is None else np.array(x0, dtype=float).reshape(4,)
        self.P = np.diag([10, 10, 10, 10]) if P0 is None else np.array(P0, dtype=float).reshape(4,4)
        self.Q = np.diag([0.05, 0.05, 0.5, 0.5]) if Q is None else np.array(Q, dtype=float).reshape(4,4)
        self.R = np.diag([2.0, 2.0]) if R is None else np.array(R, dtype=float).reshape(2,2)

    def predict(self, a, dt: float):
        ax, ay = float(a[0]), float(a[1])
        px, py, vx, vy = self.x

        px_new = px + vx * dt + 0.5 * ax * dt * dt
        py_new = py + vy * dt + 0.5 * ay * dt * dt
        vx_new = vx + ax * dt
        vy_new = vy + ay * dt
        self.x = np.array([px_new, py_new, vx_new, vy_new], dtype=float)

        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=float)

        self.P = F @ self.P @ F.T + self.Q

    def update_gps(self, z):
        z = np.asarray(z, dtype=float).reshape(2,)
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I = np.eye(4)
        self.P = (I - K @ H) @ self.P