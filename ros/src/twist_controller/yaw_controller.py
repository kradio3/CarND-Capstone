from math import atan

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_steer_angle = -max_steer_angle
        self.max_steer_angle = max_steer_angle

    def get_steering(self, target_linear_vel, target_angular_vel, current_vel):
        # TODO: find out what's the reason to adjust the target angular velocity as below
        # angular_vel = current_vel * target_angular_vel / target_linear_vel if abs(target_linear_vel) > 0. else 0.
        angular_vel = target_angular_vel

        # don't steer if angular velocity is about 0
        if abs(angular_vel) <= 0.:
            return 0.0

        # Limit angular velocity to not exceed the max lateral acceleration.
        # Read more about lateral acceleration: https://www.mrwaynesclass.com/circular/notes/corner/home.htm 
        if abs(current_vel) > 0.1:
            # lat_accel = V*V/R = V*w => w = lat_accel/V
            max_angular_vel = abs(self.max_lat_accel / current_vel)
            angular_vel = clamp(angular_vel, -max_angular_vel, max_angular_vel)

        # Find out the radius the car must follow to achieve the target angular velocity
        vel = max(current_vel, self.min_speed)
        turning_radius = vel / angular_vel # w = V/r => r = V/w

        # Knowing the radius identify what should be the steering angle
        return self.get_steer_angle(turning_radius)

    def get_steer_angle(self, turning_radius):
        # Take a loot at the bicycle model.
        # https://nabinsharma.wordpress.com/2014/01/02/kinematics-of-a-robot-bicycle-model/
        #
        # tan(wheel_angle) = wheel_base / turning_radius
        wheel_angle = atan(self.wheel_base / turning_radius)
        steer_angle = wheel_angle * self.steer_ratio
        return clamp(steer_angle, self.min_steer_angle, self.max_steer_angle)
