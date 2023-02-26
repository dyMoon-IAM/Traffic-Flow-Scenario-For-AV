#!/usr/bin/env python
# -*- coding: utf-8 -*-
from .perception.forward_object_detector import ForwardObjectDetector
from .localization.path_manager import PathManager
from .planning.adaptive_cruise_control import AdaptiveCruiseControl
from .planning.idm import IntelligentDriverModel
from .control.pure_pursuit import PurePursuit
from .control.pid import Pid
from .control.control_input import ControlInput
from .config.config import Config

from .kinematic.vehicle_kinematic_model import VehicleKinematicModel
from .vehicleVisulazation.vehicle_viz import VehicleAnimation

from .mgeo.calc_mgeo_path import mgeo_dijkstra_path, mgeo_random_path



class TrafficAgent:
    def __init__(self):
        self.config = Config()

        self.path_Planning(self.config["map"]["start_link_name"])

        self.forward_object_detector = ForwardObjectDetector(self.config["map"]["traffic_light_list"])

        self.adaptive_cruise_control = AdaptiveCruiseControl(
            vehicle_length=self.config['common']['vehicle_length'], **self.config['planning']['adaptive_cruise_control'])

        self.intelligent_driver_model = IntelligentDriverModel()
        self.pid = Pid(sampling_time=1/float(self.config['common']['sampling_rate']), **self.config['control']['pid'])
        self.pure_pursuit = PurePursuit(
            wheelbase=self.config['common']['wheelbase'], **self.config['control']['pure_pursuit']
        )

        # self.vehicle_kinematic_model = VehicleKinematicModel()
        # self.vehicle_animation = VehicleAnimation()

    def path_Planning(self, linkName):
        if self.config["map"]["use_random_path"]:
            self.random_path = mgeo_random_path(self.config["map"]["name"])
            self.path = self.random_path.plan_random_path(linkName)
            self.path_manager = PathManager(
                self.path, self.config["map"]["is_closed_path"], self.config["map"]["local_path_size"]
            )
        elif self.config["map"]["use_mgeo_path"]:
            mgeo_path = mgeo_dijkstra_path(self.config["map"]["name"])
            self.path = mgeo_path.calc_dijkstra_path(self.config["map"]["mgeo"]["start_node"], self.config["map"]["mgeo"]["end_node"])
            self.path_manager = PathManager(
                self.path, self.config["map"]["is_closed_path"], self.config["map"]["local_path_size"]
            )
        else:
            self.path = self.config["map"]["path"]
            self.path_manager = PathManager(
                self.path , self.config["map"]["is_closed_path"], self.config["map"]["local_path_size"]
            )

        self.path_manager.set_velocity_profile(**self.config['planning']['velocity_profile'])


    def execute(self, vehicle_state, dynamic_object_list, current_traffic_light):
        # 현재 위치 기반으로 local path과 planned velocity 추출
        local_path, planned_velocity, current_waypoint = self.path_manager.get_local_path(vehicle_state)

        if self.path_manager.is_drive_completion(current_waypoint):
            self.path_Planning(vehicle_state.linkName)

        # 전방 장애물 인지
        self.forward_object_detector._dynamic_object_list = dynamic_object_list
        object_info_dic_list = self.forward_object_detector.detect_object(vehicle_state)

        # # target velocity가 0이고, 일정 속도 이하일 경우 full brake를 하여 차량을 멈추도록 함.
        # if round(target_velocity) == 0 and vehicle_state.velocity < 2:
        #     acc_cmd = -1.

        # Intelligent Driver Model을 활용한 가속도 계획
        self.intelligent_driver_model.check_object(local_path, object_info_dic_list, current_traffic_light)
        self.intelligent_driver_model.check_object(local_path, object_info_dic_list, current_traffic_light)
        target_acceleration = self.intelligent_driver_model.get_acceleration(vehicle_state.velocity, planned_velocity)
        # # 가속도 제어를 위한 PID control
        # acc_cmd = self.pid.get_output(target_acceleration, vehicle_state.acceleration)

        # 경로 추종을 위한 pure pursuit control
        self.pure_pursuit.path = local_path
        self.pure_pursuit.vehicle_state = vehicle_state
        steering_cmd = self.pure_pursuit.calculate_steering_angle()


        return ControlInput(target_acceleration, steering_cmd), local_path
