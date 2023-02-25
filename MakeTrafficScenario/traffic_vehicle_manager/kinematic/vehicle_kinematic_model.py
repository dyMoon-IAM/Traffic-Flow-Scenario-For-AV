import numpy as np

class VehicleKinematicModel:
    def __init__(self, start_x=0, start_y=0, start_theta=0, start_velocity=0, vehicle_length=4.0, vehicle_width=2.0):
        self.x = start_x
        self.y = start_y
        self.theta = start_theta
        self.velocity = start_velocity
        self.L = vehicle_length
        self.W = vehicle_width


    def update(self, dt, steering_angle, acceleration):
        # Kinematic equations steering_angle(rad)
        angular_velocity = self.velocity * np.tan(steering_angle) / self.L
        self.x += self.velocity * np.cos(self.theta) * dt
        self.y += self.velocity * np.sin(self.theta) * dt
        self.theta += angular_velocity * dt
        self.velocity += acceleration * dt

        return self.x, self.y, self.theta