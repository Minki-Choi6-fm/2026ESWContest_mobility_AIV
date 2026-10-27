"""
전송 계층 — BSM 26바이트 프레임을 그대로 주고받기만 함 (내용 해석은 bsm_protocol 담당)

  UDPBroadcastTransport : 같은 Wi-Fi 안에서 UDP 브로드캐스트 (핫스팟/공유기 필요)
  EspNowTransport       : USB 시리얼로 ESP32 브릿지에 넘기고, ESP32가 ESP-NOW로 방송

v2v_node.py에서 둘 중 하나만 골라 쓰면 되고, 상위 로직은 바뀌지 않음.
"""
import socket
import time
from typing import Optional

from bsm_protocol import FRAME_LEN, SYNC, is_valid_frame


class Transport:
    name = 'base'

    def send(self, frame: bytes) -> None:
        raise NotImplementedError

    def recv(self, timeout: float) -> Optional[bytes]:
        raise NotImplementedError

    def close(self) -> None:
        pass


class UDPBroadcastTransport(Transport):
    name = 'udp'

    def __init__(self, port: int = 9000, broadcast_addr: str = '255.255.255.255'):
        self.addr = (broadcast_addr, port)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind(('', port))
        self.sock = s

    def send(self, frame: bytes) -> None:
        self.sock.sendto(frame, self.addr)

    def recv(self, timeout: float) -> Optional[bytes]:
        self.sock.settimeout(timeout)
        try:
            data, _ = self.sock.recvfrom(256)
        except socket.timeout:
            return None
        return data

    def close(self) -> None:
        self.sock.close()


class EspNowTransport(Transport):
    """
    ESP32(esp32_espnow_bridge.ino)와 USB 시리얼로 연결.
    시리얼은 바이트 스트림이라 프레임 경계가 없으므로 SYNC→26바이트→END+CRC 확인으로 재동기화.
    """
    name = 'esp-now'

    def __init__(self, port: str = '/dev/ttyUSB0', baud: int = 115200):
        import serial  # sudo apt install python3-serial

        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 0.02
        ser.dtr = False   # 포트 열 때 ESP32가 리셋/부트모드로 빠지지 않게
        ser.rts = False
        ser.open()
        time.sleep(0.3)
        ser.reset_input_buffer()
        self.ser = ser
        self.buf = bytearray()
        self.dropped_bytes = 0

    def send(self, frame: bytes) -> None:
        self.ser.write(frame)

    def _extract(self) -> Optional[bytes]:
        while True:
            i = self.buf.find(bytes([SYNC]))
            if i < 0:
                self.dropped_bytes += len(self.buf)
                self.buf.clear()
                return None
            if i > 0:
                self.dropped_bytes += i
                del self.buf[:i]
            if len(self.buf) < FRAME_LEN:
                return None
            cand = bytes(self.buf[:FRAME_LEN])
            if is_valid_frame(cand):
                del self.buf[:FRAME_LEN]
                return cand
            del self.buf[0]
            self.dropped_bytes += 1

    def recv(self, timeout: float) -> Optional[bytes]:
        deadline = time.monotonic() + timeout
        while True:
            frame = self._extract()
            if frame:
                return frame
            if time.monotonic() >= deadline:
                return None
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                self.buf += chunk

    def close(self) -> None:
        self.ser.close()
