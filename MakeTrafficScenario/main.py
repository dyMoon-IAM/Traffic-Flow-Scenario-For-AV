from network.udp_manager import UdpManager
from ScenarioModule.traffic_scenario import TrafficScenario


def main():
    traffic_flow_scearnio = TrafficScenario()
    network_manager = UdpManager(traffic_flow_scearnio)
    network_manager.execute()


if __name__ == '__main__':
    main()