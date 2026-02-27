import socket
import threading
import time
import json
from typing import Dict, Optional, Tuple

class TcpOdomPublisher:
    """
    A simple TCP server:
    - Accept multiple clients
    - Periodically broadcast latest odom as JSON line
    JSON schema example:
      {"ts": 123.4, "px": 1.0, "py": 2.0, "vx": 0.1, "vy": 0.2}
    """
    def __init__(self, host: str = "0.0.0.0", port: int = 9000, send_hz: float = 10.0):
        self.host = host
        self.port = port
        self.send_period = 1.0 / send_hz

        self._stop_evt = threading.Event()
        self._srv_sock: Optional[socket.socket] = None

        self._clients_lock = threading.Lock()
        self._clients: Dict[Tuple[str, int], socket.socket] = {}

        self._state_lock = threading.Lock()
        self._latest: Dict[str, float] = {"ts": 0.0, "px": 0.0, "py": 0.0, "vx": 0.0, "vy": 0.0}

        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)

    def start(self):
        self._srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv_sock.bind((self.host, self.port))
        self._srv_sock.listen(5)
        self._srv_sock.settimeout(0.5)

        self._accept_thread.start()
        self._send_thread.start()

    def stop(self):
        self._stop_evt.set()
        if self._srv_sock:
            try:
                self._srv_sock.close()
            except Exception:
                pass

        with self._clients_lock:
            for c in list(self._clients.values()):
                try:
                    c.close()
                except Exception:
                    pass
            self._clients.clear()

    def update_state(self, ts: float, px: float, py: float, vx: float, vy: float):
        with self._state_lock:
            self._latest = {"ts": float(ts), "px": float(px), "py": float(py), "vx": float(vx), "vy": float(vy)}

    def _accept_loop(self):
        assert self._srv_sock is not None
        while not self._stop_evt.is_set():
            try:
                client, addr = self._srv_sock.accept()
                client.setblocking(True)
                with self._clients_lock:
                    self._clients[addr] = client
                # optional greeting
                try:
                    client.sendall(b'{"type":"hello"}\n')
                except Exception:
                    pass
                print(f"[tcp] client connected: {addr}")
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception:
                continue

    def _send_loop(self):
        while not self._stop_evt.is_set():
            time.sleep(self.send_period)
            with self._state_lock:
                payload = json.dumps(self._latest) + "\n"
            data = payload.encode("utf-8")

            dead = []
            with self._clients_lock:
                for addr, c in self._clients.items():
                    try:
                        c.sendall(data)
                    except Exception:
                        dead.append(addr)

                for addr in dead:
                    try:
                        self._clients[addr].close()
                    except Exception:
                        pass
                    self._clients.pop(addr, None)
                    print(f"[tcp] client disconnected: {addr}")