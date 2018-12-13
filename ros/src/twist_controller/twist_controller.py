import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, 
                 accel_limit, wheel_radius, wheel_base, steer_ratio, 
                 max_lat_accel, max_steer_angle):

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio

        self.pid_cont = PID(0.3, 0.1, 0, 0, 0.2)
        self.vel_lpf = LowPassFilter(0.5,.02)
        self.lpf_fuel = LowPassFilter(60.0, 0.1)
        self.yaw_cont = YawController(wheel_base, steer_ratio, 4.* ONE_MPH, 
                                      max_lat_accel, max_steer_angle)
        self.last_time = rospy.get_time()

    def set_fuel(self, level):
        self.lpf_fuel.filt(level)

    def get_vehicle_mass(self):
        return self.vehicle_mass + self.lpf_fuel.get() / 100.0 * self.fuel_capacity * GAS_DENSITY

    def elapsed_time(self):
        current_time = rospy.get_time()
        elapsed_time, self.last_time = current_time - self.last_time, current_time
        return elapsed_time

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        if not dbw_enabled:
            self.pid_cont.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        throttle = self.pid_cont.step(vel_error, self.elapsed_time())
        brake = 0
        vehicle_mass = self.get_vehicle_mass()

        if linear_vel == 0 and current_vel < 0.1:
            throttle = 0
            brake = 700
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = vehicle_mass * abs(decel) * self.wheel_radius

        steering = self.yaw_cont.get_steering(linear_vel, angular_vel, current_vel)
        return throttle, brake, steering
