#!/usr/bin/env python3
import base64
import random
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8, String
import serial
import pynmea2


def _gga_to_gpgga(gga: str) -> str:
    """
    Some casters behave better with $GPGGA (legacy) than $GNGGA.
    Recompute the checksum after changing the talker ID; some SNIP/RTK2GO
    mountpoints close the stream when the upstream GGA checksum is wrong.
    """
    s = gga.strip()
    if not s.startswith("$"):
        return s
    body = s[1:].split("*", 1)[0]
    if body.startswith("GNGGA"):
        body = "GPGGA" + body[5:]
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    return f"${body}*{checksum:02X}"


def _nmea_checksum_ok(sentence: str) -> bool:
    s = sentence.strip()
    if not s.startswith("$") or "*" not in s:
        return False
    body, checksum_text = s[1:].split("*", 1)
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    try:
        expected = int(checksum_text[:2], 16)
    except ValueError:
        return False
    return checksum == expected


def _gga_summary(sentence: str) -> str:
    try:
        msg = pynmea2.parse(sentence)
        fix_q = msg.gps_qual or "?"
        sats = msg.num_sats or "?"
        lat = f"{float(msg.latitude):.7f}" if msg.latitude else "?"
        lon = f"{float(msg.longitude):.7f}" if msg.longitude else "?"
    except Exception as exc:
        return f"invalid_gga parse_error={exc}"
    checksum = "ok" if _nmea_checksum_ok(sentence) else "bad"
    talker = sentence[1:6] if sentence.startswith("$") and len(sentence) >= 6 else "?"
    return f"{talker} fixq={fix_q} sats={sats} lat={lat} lon={lon} checksum={checksum}"


class Um982NtripBridge(Node):
    """
    UM982 NTRIP bridge (RTCM -> serial) + GNSS fix publisher (GGA/GST).
    Position-only (no heading).

    Publishes:
      - gps/fix (NavSatFix)
      - gps/fix_quality (UInt8)
      - gps/rtk_state (String)
    """

    def __init__(self):
        super().__init__("um982_ntrip_bridge")

        # --- Serial params ---
        self.port = self.declare_parameter("port", "/dev/ttyGNSS").value
        self.baud = int(self.declare_parameter("baud", 115200).value)
        self.frame_id = self.declare_parameter("frame_id", "gps_link").value

        # --- NTRIP params ---
        self.ntrip_host = self.declare_parameter("ntrip_host", "rtk2go.com").value
        self.ntrip_port = int(self.declare_parameter("ntrip_port", 2101).value)
        self.mountpoint = self.declare_parameter("mountpoint", "").value
        self.username = self.declare_parameter("username", "").value
        self.password = self.declare_parameter("password", "none").value
        self.user_agent = self.declare_parameter("user_agent", "NTRIP UM982/1.0").value

        # how often to send GGA upstream to caster (seconds)
        # (0.2s helps keep RTK2GO sessions alive in some cases)
        self.gga_send_period = float(self.declare_parameter("gga_send_period", 5.0).value)
        self.send_gga = bool(self.declare_parameter("send_gga", True).value)

        # If True, convert $GNGGA -> $GPGGA before sending to caster
        self.force_gpgga = bool(self.declare_parameter("force_gpgga", True).value)

        # --- ROS pub ---
        self.pub_fix = self.create_publisher(NavSatFix, "gps/fix", 10)
        self.pub_fixq = self.create_publisher(UInt8, "gps/fix_quality", 10)
        self.pub_rtk = self.create_publisher(String, "gps/rtk_state", 10)

        # keep latest GST sigmas (meters)
        self._gst_sigma_lat = None
        self._gst_sigma_lon = None
        self._gst_sigma_alt = None
        self._last_gst_time = 0.0

        # shared state
        self._latest_gga_sentence = None
        self._latest_fix_quality = 0
        self._stop = False
        self._lock = threading.Lock()

        # open serial once
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        self.get_logger().info(f"Opened serial {self.port} @ {self.baud}")

        # start threads
        self.serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.ntrip_thread = threading.Thread(target=self._ntrip_loop, daemon=True)
        self.serial_thread.start()
        self.ntrip_thread.start()

    def destroy_node(self):
        self._stop = True
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    # ---------------- Serial parsing ----------------

    def _serial_loop(self):
        """Read NMEA lines from UM982, publish NavSatFix, store latest GGA for NTRIP."""
        buf = b""
        while rclpy.ok() and not self._stop:
            try:
                data = self.ser.read(4096)
                if not data:
                    continue
                buf += data

                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line or not line.startswith(b"$"):
                        continue

                    s = line.decode("ascii", errors="ignore").strip()

                    # Parse only sentences we care about
                    if "GGA" in s:
                        with self._lock:
                            self._latest_gga_sentence = s
                        self._handle_gga(s)
                    elif "GST" in s:
                        self._handle_gst(s)

            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                time.sleep(0.5)

    def _handle_gga(self, sentence: str):
        try:
            msg = pynmea2.parse(sentence)
        except Exception:
            return

        # GGA fix quality: 0 invalid, 1 GNSS, 2 DGPS, 4 RTK FIX, 5 RTK FLOAT
        try:
            fix_q = int(msg.gps_qual)
        except Exception:
            fix_q = 0
        self._latest_fix_quality = fix_q

        # publish fix_quality
        q = UInt8()
        q.data = max(0, min(255, fix_q))
        self.pub_fixq.publish(q)

        # publish rtk_state
        state = String()
        if fix_q == 4:
            state.data = "RTK_FIX"
        elif fix_q == 5:
            state.data = "RTK_FLOAT"
        elif fix_q == 2:
            state.data = "DGPS"
        elif fix_q == 1:
            state.data = "GNSS"
        elif fix_q == 0:
            state.data = "NO_FIX"
        else:
            state.data = f"UNKNOWN_{fix_q}"
        self.pub_rtk.publish(state)

        # Build NavSatFix
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id

        fix.latitude = float(msg.latitude) if msg.latitude else 0.0
        fix.longitude = float(msg.longitude) if msg.longitude else 0.0
        fix.altitude = float(msg.altitude) if msg.altitude else 0.0

        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.status.status = NavSatStatus.STATUS_NO_FIX if fix_q == 0 else NavSatStatus.STATUS_FIX

        # Covariance (dynamic)
        use_gst = (
            self._gst_sigma_lat is not None
            and self._gst_sigma_lon is not None
            and (time.time() - self._last_gst_time) < 2.5
        )

        if use_gst:
            sigma_lat = max(0.01, float(self._gst_sigma_lat))  # clamp min 1 cm
            sigma_lon = max(0.01, float(self._gst_sigma_lon))
            sigma_alt = max(0.02, float(self._gst_sigma_alt)) if self._gst_sigma_alt is not None else 1.0
            var_x = sigma_lon * sigma_lon
            var_y = sigma_lat * sigma_lat
            var_z = sigma_alt * sigma_alt
        else:
            # fallback mapping by fix type
            if fix_q == 4:
                sigma_xy = 0.02
            elif fix_q == 5:
                sigma_xy = 0.10
            elif fix_q == 2:
                sigma_xy = 0.50
            elif fix_q == 1:
                sigma_xy = 1.50
            else:
                sigma_xy = 10.0
            var_x = var_y = sigma_xy * sigma_xy
            var_z = (sigma_xy * 2.0) ** 2

        fix.position_covariance = [
            var_x, 0.0, 0.0,
            0.0, var_y, 0.0,
            0.0, 0.0, var_z,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub_fix.publish(fix)

    def _handle_gst(self, sentence: str):
        try:
            msg = pynmea2.parse(sentence)
        except Exception:
            return

        try:
            self._gst_sigma_lat = float(msg.std_dev_latitude) if msg.std_dev_latitude else None
            self._gst_sigma_lon = float(msg.std_dev_longitude) if msg.std_dev_longitude else None
            self._gst_sigma_alt = float(msg.std_dev_altitude) if msg.std_dev_altitude else None
            self._last_gst_time = time.time()
        except Exception:
            return

    # ---------------- NTRIP ----------------

    def _ntrip_loop(self):
        """Connect to NTRIP, stream RTCM -> serial; send GGA upstream (immediately + periodically)."""
        if not self.username or not self.mountpoint:
            self.get_logger().error(
                "NTRIP username/mountpoint is empty. Set the username and mountpoint "
                "parameters (or NTRIP_USERNAME / NTRIP_MOUNTPOINT in the environment)."
            )
            return

        reconnect_delay = 2.0
        while rclpy.ok() and not self._stop:
            sock = None
            connected_since = None
            try:
                sock = socket.create_connection((self.ntrip_host, self.ntrip_port), timeout=10.0)

                # TCP keepalive helps on Wi-Fi/NAT
                try:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                except Exception:
                    pass

                auth = base64.b64encode(f"{self.username}:{self.password}".encode()).decode()

                # SNIP/RTK2GO expects a real NTRIP client request. A plain
                # HTTP/1.1 request from curl is often rejected as "nonsense".
                req = (
                    f"GET /{self.mountpoint} HTTP/1.0\r\n"
                    f"User-Agent: {self.user_agent}\r\n"
                    f"Authorization: Basic {auth}\r\n"
                    f"\r\n"
                )
                sock.sendall(req.encode("ascii"))

                # Read initial response (NTRIP can be ICY 200 OK)
                sock.settimeout(2.0)
                data = b""
                start = time.time()
                while len(data) < 16384 and (time.time() - start) < 2.0:
                    try:
                        chunk = sock.recv(1024)
                        if not chunk:
                            raise RuntimeError("NTRIP closed during header")
                        data += chunk
                        if b"\r\n\r\n" in data:
                            break
                        if data.startswith(b"ICY 200 OK") and b"\r\n" in data:
                            break
                    except socket.timeout:
                        break

                header = data
                rest = b""

                if b"\r\n\r\n" in data:
                    header, rest = data.split(b"\r\n\r\n", 1)
                else:
                    if data.startswith(b"ICY 200 OK") and b"\r\n" in data:
                        first, tail = data.split(b"\r\n", 1)
                        header = first
                        rest = tail

                if (b"200" not in header) and (b"ICY 200 OK" not in header):
                    raise RuntimeError(f"NTRIP rejected/unknown response: {header[:200]!r}")

                self.get_logger().info("NTRIP connected, streaming RTCM.")
                connected_since = time.time()
                sock.settimeout(2.0)

                # Forward any bytes already received after the header
                if rest:
                    self.ser.write(rest)

                last_gga_sent = 0.0

                # Send one GGA immediately after connect. Many RTK2GO/SNIP
                # mountpoints close rover sessions if they do not receive GGA
                # promptly; send the latest sentence even before RTK quality
                # improves as long as the receiver is producing GGA.
                if self.send_gga:
                    with self._lock:
                        gga0 = self._latest_gga_sentence
                    if gga0:
                        gga_send = _gga_to_gpgga(gga0) if self.force_gpgga else gga0.strip()
                        try:
                            sock.sendall((gga_send + "\r\n").encode("ascii", errors="ignore"))
                            last_gga_sent = time.time()
                            self.get_logger().info(
                                f"Sent initial GGA to NTRIP caster: {_gga_summary(gga_send)}"
                            )
                        except Exception:
                            pass
                    else:
                        self.get_logger().warn("No GGA available yet; caster may close NTRIP stream.")

                # Debug: RTCM throughput
                rtcm_bytes = 0
                t0 = time.time()

                while rclpy.ok() and not self._stop:
                    now = time.time()

                    # Periodically send GGA upstream whenever the receiver is
                    # producing GGA. The caster may need it even for non-RTK
                    # fix quality to choose nearby corrections.
                    if self.send_gga and (now - last_gga_sent) >= self.gga_send_period:
                        with self._lock:
                            gga = self._latest_gga_sentence
                        if gga:
                            gga_send = _gga_to_gpgga(gga) if self.force_gpgga else gga.strip()
                            try:
                                sock.sendall((gga_send + "\r\n").encode("ascii", errors="ignore"))
                                last_gga_sent = now
                                self.get_logger().debug(
                                    f"Sent periodic GGA to NTRIP caster: {_gga_summary(gga_send)}"
                                )
                            except Exception:
                                pass

                    try:
                        rtcm = sock.recv(4096)
                        if not rtcm:
                            raise RuntimeError("NTRIP stream closed")
                        self.ser.write(rtcm)

                        rtcm_bytes += len(rtcm)
                        if now - t0 >= 5.0:
                            kbps = (rtcm_bytes / max(0.001, (now - t0))) / 1024.0
                            self.get_logger().info(f"RTCM rx ~{kbps:.1f} KB/s")
                            rtcm_bytes = 0
                            t0 = now

                    except socket.timeout:
                        if now - t0 >= 5.0:
                            kbps = (rtcm_bytes / max(0.001, (now - t0))) / 1024.0
                            self.get_logger().info(f"RTCM rx ~{kbps:.1f} KB/s")
                            rtcm_bytes = 0
                            t0 = now
                        continue

            except Exception as e:
                if connected_since is not None and (time.time() - connected_since) >= 30.0:
                    reconnect_delay = 2.0
                else:
                    reconnect_delay = min(20.0, reconnect_delay * 1.5)
                sleep_sec = reconnect_delay + random.uniform(0.0, 1.0)
                self.get_logger().warn(
                    f"NTRIP error: {e}. Reconnecting in {sleep_sec:.1f}s..."
                )
                time.sleep(sleep_sec)
            finally:
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass


def main():
    rclpy.init()
    node = Um982NtripBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
