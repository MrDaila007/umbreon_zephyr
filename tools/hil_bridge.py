#!/usr/bin/env python3
"""
hil_bridge.py — Unified HIL transport without WiFi module.

Modes:
  - real: dashboard <-> TCP bridge <-> RP2350 UART
  - sim:  dashboard <-> TCP bridge <-> simulation TCP bridge
  - dual: two TCP endpoints in one process:
          * real endpoint  -> RP2350 UART
          * sim endpoint   -> simulation TCP bridge
"""

from __future__ import annotations

import argparse
import queue
import socket
import threading
import time
from typing import Callable, Optional

try:
    import serial  # type: ignore
except Exception:  # pragma: no cover
    serial = None


class ClientHub:
    def __init__(self, host: str, port: int, name: str, on_line: Callable[[str], None]):
        self.host = host
        self.port = port
        self.name = name
        self._on_line = on_line
        self._srv: Optional[socket.socket] = None
        self._clients: list[socket.socket] = []
        self._clients_lock = threading.Lock()
        self._running = False

    def start(self) -> None:
        self._running = True
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(8)
        self._srv.settimeout(0.5)
        threading.Thread(target=self._accept_loop, daemon=True).start()
        print(f"[hub:{self.name}] listening on {self.host}:{self.port}")

    def stop(self) -> None:
        self._running = False
        if self._srv:
            try:
                self._srv.close()
            except OSError:
                pass
        with self._clients_lock:
            for c in self._clients:
                try:
                    c.close()
                except OSError:
                    pass
            self._clients.clear()

    def broadcast(self, line: str) -> None:
        data = line.encode("ascii", errors="replace")
        dead: list[socket.socket] = []
        with self._clients_lock:
            for c in self._clients:
                try:
                    c.sendall(data)
                except OSError:
                    dead.append(c)
            for c in dead:
                try:
                    c.close()
                except OSError:
                    pass
                if c in self._clients:
                    self._clients.remove(c)

    def _accept_loop(self) -> None:
        assert self._srv is not None
        while self._running:
            try:
                conn, addr = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            conn.settimeout(0.5)
            with self._clients_lock:
                self._clients.append(conn)
            print(f"[hub:{self.name}] client connected: {addr[0]}:{addr[1]}")
            threading.Thread(target=self._client_loop, args=(conn,), daemon=True).start()

    def _client_loop(self, conn: socket.socket) -> None:
        buf = b""
        while self._running:
            try:
                data = conn.recv(4096)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("ascii", errors="replace").strip()
                    if line:
                        self._on_line(line + "\n")
            except socket.timeout:
                continue
            except OSError:
                break

        with self._clients_lock:
            if conn in self._clients:
                self._clients.remove(conn)
        try:
            conn.close()
        except OSError:
            pass


class SerialEndpoint:
    def __init__(self, port: str, baud: int, on_line: Callable[[str], None], reconnect_s: float = 1.0):
        self.port = port
        self.baud = baud
        self._on_line = on_line
        self._reconnect_s = reconnect_s
        self._txq: "queue.Queue[str]" = queue.Queue()
        self._running = False

    def start(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install pyserial")
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        print(f"[serial] endpoint on {self.port} @ {self.baud}")

    def stop(self) -> None:
        self._running = False

    def send(self, line: str) -> None:
        self._txq.put(line)

    def _loop(self) -> None:
        while self._running:
            ser = None
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.2)
                print(f"[serial] connected: {self.port}")
                while self._running and ser.is_open:
                    raw = ser.readline()
                    if raw:
                        line = raw.decode("ascii", errors="replace")
                        self._on_line(line if line.endswith("\n") else line + "\n")

                    for _ in range(64):
                        try:
                            out = self._txq.get_nowait()
                        except queue.Empty:
                            break
                        ser.write(out.encode("ascii", errors="replace"))
            except Exception as e:  # pragma: no cover
                print(f"[serial] reconnecting ({e})")
                time.sleep(self._reconnect_s)
            finally:
                if ser:
                    try:
                        ser.close()
                    except Exception:
                        pass


class TcpEndpoint:
    def __init__(self, host: str, port: int, on_line: Callable[[str], None], reconnect_s: float = 1.0):
        self.host = host
        self.port = port
        self._on_line = on_line
        self._reconnect_s = reconnect_s
        self._txq: "queue.Queue[str]" = queue.Queue()
        self._running = False

    def start(self) -> None:
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        print(f"[tcp] upstream {self.host}:{self.port}")

    def stop(self) -> None:
        self._running = False

    def send(self, line: str) -> None:
        self._txq.put(line)

    def _loop(self) -> None:
        while self._running:
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2.0)
                sock.connect((self.host, self.port))
                sock.settimeout(0.2)
                print(f"[tcp] connected: {self.host}:{self.port}")
                buf = b""
                while self._running:
                    try:
                        data = sock.recv(4096)
                        if not data:
                            raise OSError("upstream closed")
                        buf += data
                        while b"\n" in buf:
                            raw, buf = buf.split(b"\n", 1)
                            line = raw.decode("ascii", errors="replace")
                            if line:
                                self._on_line(line + "\n")
                    except socket.timeout:
                        pass

                    for _ in range(64):
                        try:
                            out = self._txq.get_nowait()
                        except queue.Empty:
                            break
                        sock.sendall(out.encode("ascii", errors="replace"))
            except Exception as e:  # pragma: no cover
                print(f"[tcp] reconnecting ({e})")
                time.sleep(self._reconnect_s)
            finally:
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass


def run_real(args: argparse.Namespace) -> None:
    hub = ClientHub(args.listen_host, args.listen_port, "real", lambda line: serial_ep.send(line))
    serial_ep = SerialEndpoint(args.serial_port, args.serial_baud, hub.broadcast, args.reconnect_s)
    hub.start()
    serial_ep.start()
    while True:
        time.sleep(1.0)


def run_sim(args: argparse.Namespace) -> None:
    hub = ClientHub(args.listen_host, args.listen_port, "sim", lambda line: sim_ep.send(line))
    sim_ep = TcpEndpoint(args.sim_host, args.sim_port, hub.broadcast, args.reconnect_s)
    hub.start()
    sim_ep.start()
    while True:
        time.sleep(1.0)


def run_dual(args: argparse.Namespace) -> None:
    real_hub = ClientHub(args.listen_host, args.listen_port, "real", lambda line: serial_ep.send(line))
    sim_hub = ClientHub(args.listen_host, args.listen_sim_port, "sim", lambda line: sim_ep.send(line))
    serial_ep = SerialEndpoint(args.serial_port, args.serial_baud, real_hub.broadcast, args.reconnect_s)
    sim_ep = TcpEndpoint(args.sim_host, args.sim_port, sim_hub.broadcast, args.reconnect_s)
    real_hub.start()
    sim_hub.start()
    serial_ep.start()
    sim_ep.start()
    print(f"[dual] real endpoint: {args.listen_host}:{args.listen_port}")
    print(f"[dual] sim  endpoint: {args.listen_host}:{args.listen_sim_port}")
    while True:
        time.sleep(1.0)


def main() -> None:
    p = argparse.ArgumentParser(description="HIL bridge without WiFi module")
    p.add_argument("--mode", choices=["real", "sim", "dual"], required=True)
    p.add_argument("--listen-host", default="127.0.0.1")
    p.add_argument("--listen-port", type=int, default=8023)
    p.add_argument("--listen-sim-port", type=int, default=8024)
    p.add_argument("--serial-port", default="/dev/ttyUSB0")
    p.add_argument("--serial-baud", type=int, default=115200)
    p.add_argument("--sim-host", default="127.0.0.1")
    p.add_argument("--sim-port", type=int, default=8023)
    p.add_argument("--reconnect-s", type=float, default=1.0)
    args = p.parse_args()

    try:
        if args.mode == "real":
            run_real(args)
        elif args.mode == "sim":
            run_sim(args)
        else:
            run_dual(args)
    except KeyboardInterrupt:
        print("\n[hil-bridge] stopped")


if __name__ == "__main__":
    main()
