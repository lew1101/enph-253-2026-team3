from dataclasses import dataclass, field

from udp_client import *
import threading

# from inputs import devices


@dataclass
class RobotState:
    left: float = 0.0
    right: float = 0.0
    estop: bool = False
    ping_requested: bool = False

    lock: threading.Lock = field(default_factory=threading.Lock, repr=False)


def main():
    # Connect your computer to the ESP32 AP first:
    # SSID: ESP32-Robot, password: robot1234
    with RobotUdpClient() as robot:
        print("send ping")
        robot.send_ping()
        robot.send_command(drivetrain_l=0.2, drivetrain_r=0.2)

        while True:
            try:
                packet, addr = robot.receive_packet()
                print("yay packet")
            except socket.timeout:
                continue
            except TlvFormatError as tlv_e:
                print(f"bad packet: {tlv_e}")
                continue

            if packet.header.packet_type == PacketType.TELEMETRY:
                print(addr, RobotTelemetry.from_packet(packet))
            elif packet.header.packet_type == PacketType.PONG:
                print(addr, "PONG", packet.header)


if __name__ == "__main__":
    main()
