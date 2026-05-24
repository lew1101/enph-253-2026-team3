import socket
import time
from dataclasses import dataclass

from tlv_packet import *


@dataclass(frozen=True)
class RobotTelemetry:
    battery_v: float = 0.0
    drivetrain_l: float = 0.0
    drivetrain_r: float = 0.0
    err: int = 0
    sequence: int = 0
    tick: int = 0

    @staticmethod
    def from_packet(packet: Packet) -> "RobotTelemetry":
        if packet.header.packet_type != PacketType.TELEMETRY:
            raise TlvFormatError(
                f"expected TELEMETRY, got {packet.header.packet_type.name}"
            )

        return RobotTelemetry(
            battery_v=packet.get_float(TlvType.BATT_V, 0.0) or 0.0,
            drivetrain_l=packet.get_float(TlvType.DRIVETRAIN_L, 0.0) or 0.0,
            drivetrain_r=packet.get_float(TlvType.DRIVETRAIN_R, 0.0) or 0.0,
            err=packet.get_i32(TlvType.ERR, 0) or 0,
            sequence=packet.header.packet_seq,
            tick=packet.header.tick,
        )


@dataclass(frozen=True)
class RobotCommand:
    drivetrain_l: float = 0.0
    drivetrain_r: float = 0.0
    estop: int = 0
    sequence: int = 0
    tick: int = 0


def build_command_packet(command: RobotCommand) -> bytes:
    return build_packet(
        packet_type=PacketType.COMMAND,
        sequence=command.sequence,
        tick=command.tick,
        fields=(
            TlvField.pack_float(TlvType.CMD_DRIVETRAIN_L, command.drivetrain_l),
            TlvField.pack_float(TlvType.CMD_DRIVETRAIN_R, command.drivetrain_r),
            TlvField.pack_u8(TlvType.CMD_ESTOP, command.estop),
        ),
    )


def build_ping_packet(sequence: int, tick: int) -> bytes:
    return build_packet(PacketType.PING, sequence=sequence, tick=tick)


class RobotUdpClient:
    def __init__(
        self,
        robot_ip: str = "192.168.4.1",
        robot_port: int = 4210,
        bind_ip: str = "0.0.0.0",
        bind_port: int = 0,
        timeout_s: float = 0.25,
    ) -> None:
        self.robot_addr = (robot_ip, robot_port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((bind_ip, bind_port))
        self._sock.settimeout(timeout_s)
        self._sequence = 0

    def close(self) -> None:
        self._sock.close()

    def __enter__(self) -> "RobotUdpClient":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def next_sequence(self) -> int:
        sequence = self._sequence
        self._sequence = (self._sequence + 1) & 0xFFFF
        return sequence

    @staticmethod
    def tick_ms() -> int:
        return int(time.monotonic() * 1000) & 0xFFFFFFFF

    def send_command(
        self, drivetrain_l: float, drivetrain_r: float, estop: int = 0
    ) -> int:
        sequence = self.next_sequence()
        packet = build_command_packet(
            RobotCommand(
                drivetrain_l=drivetrain_l,
                drivetrain_r=drivetrain_r,
                estop=estop,
                sequence=sequence,
                tick=self.tick_ms(),
            )
        )
        self._sock.sendto(packet, self.robot_addr)
        return sequence

    def send_ping(self) -> int:
        sequence = self.next_sequence()
        self._sock.sendto(build_ping_packet(sequence, self.tick_ms()), self.robot_addr)
        return sequence

    def receive_packet(self, max_bytes: int = 2048) -> tuple[Packet, tuple[str, int]]:
        data, addr = self._sock.recvfrom(max_bytes)
        return parse_packet(data), addr

    def receive_telemetry(self, max_bytes: int = 2048) -> RobotTelemetry:
        packet, _ = self.receive_packet(max_bytes)
        return RobotTelemetry.from_packet(packet)
