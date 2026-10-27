"""
V2V 전송 계층 추상화.

bsm_protocol.py(프레임 인코딩/디코딩)는 이 파일의 내용을 전혀 모릅니다.
지금은 UDPBroadcastTransport만 구현되어 있고, 나중에 ESP32(ESP-NOW)로 넘어갈 때는
동일한 Transport 인터페이스(send_bytes / recv_bytes)를 구현하는
EspNowTransport 클래스를 하나 더 만들어서 교체하기만 하면 됩니다.
(v2v_node.py 등 상위 코드는 한 줄도 안 바꿔도 됩니다)
"""

import socket
import select
import time
from abc import ABC, abstractmethod

from bsm_protocol import SYNC_BYTE, END_BYTE, FRAME_SIZE


class Transport(ABC):
    """모든 전송 구현체가 따라야 하는 공통 인터페이스."""

    @abstractmethod
    def send_bytes(self, data: bytes) -> None:
        """프레임 1개를 브로드캐스트/송신합니다."""
        raise NotImplementedError

    @abstractmethod
    def recv_bytes(self, timeout: float = 0.0) -> bytes | None:
        """프레임 1개를 수신합니다. timeout(초) 안에 못 받으면 None을 반환합니다."""
        raise NotImplementedError

    def close(self) -> None:
        pass


class UDPBroadcastTransport(Transport):
    """RPi 내장 Wi-Fi로 UDP 브로드캐스트를 주고받는 구현체 (현재 사용).

    세 라즈베리파이가 전부 같은 Wi-Fi 네트워크(같은 서브넷)에 붙어있어야
    브로드캐스트가 서로에게 도달합니다.
    """

    def __init__(self, port: int = 9000, broadcast_addr: str = "255.255.255.255",
                 recv_buf_size: int = 128):
        self.port = port
        self.broadcast_addr = broadcast_addr
        self.recv_buf_size = recv_buf_size

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("", self.port))
        self.sock.setblocking(False)

    def send_bytes(self, data: bytes) -> None:
        self.sock.sendto(data, (self.broadcast_addr, self.port))

    def recv_bytes(self, timeout: float = 0.0) -> bytes | None:
        ready, _, _ = select.select([self.sock], [], [], timeout)
        if not ready:
            return None
        try:
            data, _addr = self.sock.recvfrom(self.recv_buf_size)
            return data
        except BlockingIOError:
            return None

    def close(self) -> None:
        self.sock.close()


class EspNowTransport(Transport):
    """RPi <-> ESP32(USB-Serial) <-> ESP-NOW 브로드캐스트 구현체.

    RPi는 인코딩된 26바이트 BSM 프레임을 그대로 UART(USB-Serial)로 ESP32에
    넘겨주고, ESP32 펌웨어(esp32_espnow_bridge.ino)가 그걸 그대로 ESP-NOW로
    브로드캐스트합니다. 반대로 ESP32가 ESP-NOW로 받은 프레임도 그대로 UART로
    RPi에 올려줍니다.

    UART는 패킷 경계가 없는 바이트 스트림이라, bsm_protocol의 SYNC(0xAA)/
    END(0x55) 식별자를 이용해 프레임 경계를 다시 찾아냅니다(재동기화).
    CRC 검증 자체는 bsm_protocol.decode_bsm()이 상위 계층에서 이미 하고 있으니
    여기서는 "26바이트를 SYNC~END로 잘라내는 것"까지만 책임집니다.
    """

    def __init__(self, serial_port: str, baudrate: int = 115200):
        try:
            import serial  # pyserial
        except ImportError as e:
            raise ImportError(
                "pyserial이 필요합니다: pip install pyserial --break-system-packages"
            ) from e

        self._serial_module = serial
        self.ser = serial.Serial(serial_port, baudrate, timeout=0)
        self._rx_buf = bytearray()

    def send_bytes(self, data: bytes) -> None:
        assert len(data) == FRAME_SIZE, f"프레임 길이 불일치: {len(data)} != {FRAME_SIZE}"
        self.ser.write(data)

    def recv_bytes(self, timeout: float = 0.0) -> bytes | None:
        deadline = time.monotonic() + timeout

        while True:
            # 시리얼 버퍼에 쌓인 걸 전부 읽어서 내부 버퍼에 누적
            waiting = self.ser.in_waiting
            if waiting:
                self._rx_buf.extend(self.ser.read(waiting))

            frame = self._extract_frame()
            if frame is not None:
                return frame

            if time.monotonic() >= deadline:
                return None
            time.sleep(0.005)  # 5ms 폴링 (바쁜 대기 방지)

    def _extract_frame(self) -> bytes | None:
        """내부 버퍼에서 SYNC~END로 유효해 보이는 26바이트를 찾아 꺼냅니다.

        CRC까지는 여기서 안 보고(그건 decode_bsm 몫), SYNC/END 위치만 맞으면
        일단 꺼내줍니다. 진짜 손상된 프레임이면 상위(decode_bsm)에서
        BSMDecodeError로 걸러집니다.
        """
        while True:
            sync_idx = self._rx_buf.find(bytes([SYNC_BYTE]))
            if sync_idx == -1:
                self._rx_buf.clear()
                return None

            # SYNC 앞의 쓰레기 바이트는 버림
            if sync_idx > 0:
                del self._rx_buf[:sync_idx]

            if len(self._rx_buf) < FRAME_SIZE:
                return None  # 아직 26바이트가 다 안 모임, 다음 호출을 기다림

            candidate = bytes(self._rx_buf[:FRAME_SIZE])
            if candidate[-1] == END_BYTE:
                del self._rx_buf[:FRAME_SIZE]
                return candidate

            # END 위치가 안 맞으면 이 SYNC는 가짜(우연히 0xAA가 낀 것) -> 한 바이트 버리고 재탐색
            del self._rx_buf[0]

    def close(self) -> None:
        self.ser.close()
