from network.udp_manager import UdpManager
from traffic_vehicle_manager.traffic_agent_driving import TrafficAgent


def main():
    autonomous_driving = TrafficAgent()
    udp_manager = UdpManager(autonomous_driving)
    udp_manager.execute()


if __name__ == '__main__':
    main()
