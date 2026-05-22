from udp_client import *
import input


def main():
    # Connect your computer to the ESP32 AP first:
    # SSID: ESP32-Robot, password: robot1234
    with RobotUdpClient() as robot:
        robot.send_ping()
        # robot.send_command(drivetrain_l=0.2, drivetrain_r=0.2)

        while True:
            try:
                packet, addr = robot.receive_packet()
            except socket.timeout:
                continue
            except TlvFormatError as exc:
                print(f"bad packet: {exc}")
                continue

            if packet.header.packet_type == PacketType.TELEMETRY:
                print(addr, RobotTelemetry.from_packet(packet))
            elif packet.header.packet_type == PacketType.PONG:
                print(addr, "PONG", packet.header)


if __name__ == "__main__":
    main()
