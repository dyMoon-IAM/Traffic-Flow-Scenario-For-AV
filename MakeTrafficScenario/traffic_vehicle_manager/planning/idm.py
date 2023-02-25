import numpy as np
import math
from ..vehicle_state import VehicleState


class IntelligentDriverModel:
    def __init__(self, vehicle_length=2.8, maxAcceleration=1., maxDeceleration=3., minSpace=2., targetVel=60.,
                 headWayTime=1., accelExponent=4.0):

        self.maxAcceleration = maxAcceleration  # maximum acceleration      (m/s^2)
        self.maxDeceleration = maxDeceleration  # maximum deceleration      (m/s^2)
        self.minSpace = minSpace  # minimum Safety distance   (m)
        self.targetVel = targetVel / 3.6  # Target Velocity           (m/s)
        self.headWayTime = headWayTime  # minimum time headway      (s)
        self.accelExponent = accelExponent  # acceleration exponent     ( )

        self._vehicle_state = VehicleState()
        self.vehicle_length = vehicle_length

        self.object_type = None
        self.object_distance = 0
        self.object_velocity = 0

    def calc_distance(self):
        """
        S* = minSpace + max(0, Tv + v*(v-vl)/2*sqrt(maxAcceleration,maxDeceleration))

        """
        headWayDistance = (self.headWayTime * self.targetVel)
        relativeVelocityDistance = ((self.targetVel * (self.targetVel - self.object_velocity)) / (
                    2 * math.sqrt(self.maxAcceleration * self.maxDeceleration)))

        return self.minSpace + max(0, headWayDistance + relativeVelocityDistance)

    def get_acceleration(self, currVel):
        # Calculate desired gap and acceleration based on IDM equations
        distance = self.calc_distance()
        idmAccelleration = self.maxAcceleration * (
                    1 - (currVel / self.targetVel) ** self.accelExponent - (distance / self.object_distance) ** 2)

        # Apply acceleration limits and return result
        return max(-self.maxDeceleration,
                   min(idmAccelleration, self.maxAcceleration * (1 - (currVel / self.targetVel) ** self.accelExponent)))

    def check_object(self, local_path, object_info_dic_list, current_traffic_light):
        """경로상의 장애물 유무 확인 (차량, 사람, 정지선 신호)"""
        self.object_distance = float('inf')
        self.object_velocity = float('inf')
        self.object_type = None
        min_relative_distance = float('inf')
        for object_info_dic in object_info_dic_list:
            object_info = object_info_dic['object_info']
            local_position = object_info_dic['local_position']

            object_type = object_info.type
            if object_type == 0:
                distance_threshold = 4.35
            elif object_type in [1, 2]:
                distance_threshold = 2.5
            elif object_type == 3:
                if current_traffic_light and object_info.name == current_traffic_light[0] and not current_traffic_light[
                                                                                                      1] in [16, 48]:
                    distance_threshold = 9
                else:
                    continue
            else:
                continue

            for point in local_path:
                distance_from_path = point.distance(object_info.position)

                if distance_from_path < distance_threshold:
                    relative_distance = local_position.distance()
                    if relative_distance < min_relative_distance:
                        min_relative_distance = relative_distance
                        self.object_type = object_type
                        self.object_distance = relative_distance - self.vehicle_length
                        self.object_velocity = object_info.velocity
