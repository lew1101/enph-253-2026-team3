import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable

MAGIC_NUMBER = 0xABCD
VERSION = 1

# Matches C++ packed TlvPacketHeader:
# uint16_t magic;
# uint8_t  version;
# uint8_t  packet_type;
# uint16_t packet_len;
# uint16_t packet_seq;
# uint32_t tick;
HEADER_STRUCT = struct.Struct("<HBBHHI")
TLV_HEADER_STRUCT = struct.Struct("<BH")


class PacketType(IntEnum):
    TELEMETRY = 1
    COMMAND = 2
    ACK = 3
    PING = 4
    PONG = 5


class TlvType(IntEnum):
    BATT_V = 1
    DRIVETRAIN_L = 2
    DRIVETRAIN_R = 3
    LOG = 4
    ERR = 5

    CMD_DRIVETRAIN_L = 17
    CMD_DRIVETRAIN_R = 18
    CMD_ESTOP = 19


class TlvFormatError(ValueError):
    pass


@dataclass(frozen=True)
class PacketHeader:
    packet_type: PacketType
    packet_len: int
    packet_seq: int
    tick: int
    magic: int = MAGIC_NUMBER
    version: int = VERSION

    @classmethod
    def unpack_from(cls, data: bytes | bytearray | memoryview) -> "PacketHeader":
        if len(data) < HEADER_STRUCT.size:
            raise TlvFormatError("packet is smaller than header")

        magic, version, packet_type, packet_len, packet_seq, tick = (
            HEADER_STRUCT.unpack_from(data)
        )

        if magic != MAGIC_NUMBER:
            raise TlvFormatError(f"bad magic: 0x{magic:04X}")
        if version != VERSION:
            raise TlvFormatError(f"bad version: {version}")
        if packet_len != len(data):
            raise TlvFormatError(f"packet_len={packet_len}, actual_len={len(data)}")

        try:
            packet_type_enum = PacketType(packet_type)
        except ValueError as exc:
            raise TlvFormatError(f"unknown packet type: {packet_type}") from exc

        return cls(
            packet_type=packet_type_enum,
            packet_len=packet_len,
            packet_seq=packet_seq,
            tick=tick,
            magic=magic,
            version=version,
        )

    def pack(self) -> bytes:
        return HEADER_STRUCT.pack(
            self.magic,
            self.version,
            int(self.packet_type),
            self.packet_len,
            self.packet_seq,
            self.tick,
        )


@dataclass(frozen=True)
class TlvField:
    type: TlvType | int
    payload: bytes

    @property
    def payload_len(self) -> int:
        return len(self.payload)

    @staticmethod
    def pack_float(field_type: TlvType, value: float) -> "TlvField":
        return TlvField(field_type, struct.pack("<f", float(value)))

    @staticmethod
    def pack_u8(field_type: TlvType, value: int) -> "TlvField":
        if not 0 <= value <= 0xFF:
            raise ValueError("u8 out of range")
        return TlvField(field_type, struct.pack("<B", value))

    @staticmethod
    def pack_i32(field_type: TlvType, value: int) -> "TlvField":
        return TlvField(field_type, struct.pack("<i", value))

    @staticmethod
    def pack_string(field_type: TlvType, value: str) -> "TlvField":
        return TlvField(field_type, value.encode("utf-8"))

    def pack(self) -> bytes:
        if self.payload_len > 0xFFFF:
            raise ValueError("TLV payload too large")
        return TLV_HEADER_STRUCT.pack(int(self.type), self.payload_len) + self.payload


@dataclass(frozen=True)
class Packet:
    header: PacketHeader
    fields: tuple[TlvField, ...] = ()

    def get(self, field_type: TlvType) -> TlvField | None:
        for field in self.fields:
            if int(field.type) == int(field_type):
                return field
        return None

    def require(self, field_type: TlvType) -> TlvField:
        field = self.get(field_type)
        if field is None:
            raise TlvFormatError(f"missing TLV field: {field_type.name}")
        return field

    def get_float(
        self, field_type: TlvType, default: float | None = None
    ) -> float | None:
        field = self.get(field_type)
        if field is None:
            return default
        if field.payload_len != 4:
            raise TlvFormatError(
                f"{field_type.name} length must be 4, got {field.payload_len}"
            )
        return struct.unpack("<f", field.payload)[0]

    def get_u8(self, field_type: TlvType, default: int | None = None) -> int | None:
        field = self.get(field_type)
        if field is None:
            return default
        if field.payload_len != 1:
            raise TlvFormatError(
                f"{field_type.name} length must be 1, got {field.payload_len}"
            )
        return field.payload[0]

    def get_i32(self, field_type: TlvType, default: int | None = None) -> int | None:
        field = self.get(field_type)
        if field is None:
            return default
        if field.payload_len != 4:
            raise TlvFormatError(
                f"{field_type.name} length must be 4, got {field.payload_len}"
            )
        return struct.unpack("<i", field.payload)[0]

    def get_string(self, field_type: TlvType, default: str | None = None) -> str | None:
        field = self.get(field_type)
        if field is None:
            return default
        return field.payload.decode("utf-8", errors="replace")


def build_packet(
    packet_type: PacketType,
    sequence: int,
    tick: int,
    fields: Iterable[TlvField] = (),
) -> bytes:
    body = b"".join(field.pack() for field in fields)
    packet_len = HEADER_STRUCT.size + len(body)

    if not 0 <= sequence <= 0xFFFF:
        raise ValueError("sequence must fit uint16_t")
    if not 0 <= tick <= 0xFFFFFFFF:
        raise ValueError("tick must fit uint32_t")
    if packet_len > 0xFFFF:
        raise ValueError("packet too large")

    header = PacketHeader(
        packet_type=packet_type,
        packet_len=packet_len,
        packet_seq=sequence,
        tick=tick,
    )
    return header.pack() + body


def parse_packet(data: bytes | bytearray | memoryview) -> Packet:
    data = bytes(data)
    header = PacketHeader.unpack_from(data)

    fields: list[TlvField] = []
    offset = HEADER_STRUCT.size

    while offset < len(data):
        if len(data) - offset < TLV_HEADER_STRUCT.size:
            raise TlvFormatError("truncated TLV header")

        tlv_type_raw, payload_len = TLV_HEADER_STRUCT.unpack_from(data, offset)
        offset += TLV_HEADER_STRUCT.size

        end = offset + payload_len
        if end > len(data):
            raise TlvFormatError("truncated TLV payload")

        try:
            tlv_type: TlvType | int = TlvType(tlv_type_raw)
        except ValueError:
            tlv_type = tlv_type_raw

        fields.append(TlvField(tlv_type, data[offset:end]))
        offset = end

    return Packet(header=header, fields=tuple(fields))
