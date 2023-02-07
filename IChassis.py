import numpy as np
import scipy.io as spio

from driver import Driver
from controller import Controller
from vehicle import Vehicle
from battery_pack import Battery
from electric_motor import ElectricMotor
from gearbox import Gearbox
from front_brakes import FrontBrakes
from rear_brakes import RearBrakes
from front_wheels import FrontWheels
from rear_wheels import RearWheels

delta_t = 1
vehicle_mass = 3000 * 0.453592  # in kg

k_p = 0.1
k_i = 0.03  # 0.6
k_d = 0  # 2

c_drag = 0.24
frontal_area = 3.5
em_efficiency = 0.95
em_gearbox_ratio = 18.63

brake_rear_proportion = 0.4
brake_front_proportion = 0.6

tire_radius = 0.465

class IChassis:
    def __init__(self):
        self.driver = Driver(k_p, k_i, k_d, delta_t)
        self.vehicle = Vehicle(vehicle_mass, c_drag, frontal_area, delta_t)
        self.battery = Battery(0.6, 3, 120, delta_t)
        self.em = ElectricMotor()
        self.em_gearbox = Gearbox(em_efficiency, em_gearbox_ratio)
        self.rear_brakes = RearBrakes(brake_rear_proportion, tire_radius)
        self.front_brakes = FrontBrakes(brake_front_proportion, tire_radius)
        self.rear_wheels = RearWheels(tire_radius)
        self.front_wheels = FrontWheels(tire_radius)
        
        self.v_out = []
        self.power_req_out = []
        self.battery_power_out = []
        self.em_torque_out = []
        self.front_wheel_torque_out = []
        self.rear_wheel_torque_out = []
        self.force_at_wheel_out = []
        self.front_brake_torque_out = []
        self.rear_brake_torque_out = []
        self.alpha = []
        self.beta = []
        self.v_p = []
        self.v_i = []
    
    def drive(self, commanded_velocity: float):
        """ Simulates a single second of a drive step and adds it to the current drive history. 
        commanded_velocity - Real number in m/s
        """

        driver_out = self.driver.compute_step(commanded_velocity, self.vehicle.velocity)

        # compute necessary battery power
        power_req = self.driver.alpha * self.battery.compute_max_power()
        self.power_req_out.append(power_req)

        b_out = self.battery.compute_step(power_req)
        self.battery_power_out.append(b_out['pack_power'])

        self.em.compute_step(self.battery.p_pack, self.em_gearbox.em_force_w)
        self.em_torque_out.append(self.em.em_torque)

        self.rear_brakes.compute_step(self.rear_wheels.wheel_torque, self.driver.beta)
        self.front_brakes.compute_step(self.front_wheels.wheel_torque, self.driver.beta)

        self.em_gearbox.compute_step(self.em.em_torque, self.front_brakes.front_brake_w)

        self.front_wheels.compute_step(
            self.em_gearbox.torque_out / 2, self.front_brakes.brake_torque, self.vehicle.velocity)

        self.rear_wheels.compute_step(
            self.em_gearbox.torque_out / 2, self.rear_brakes.brake_torque, self.vehicle.velocity)

        vehicle_data = self.vehicle.compute_step(self.front_wheels.force_at_wheel +
                                            self.rear_wheels.force_at_wheel, 0)

        self.front_wheel_torque_out.append(self.front_wheels.wheel_torque)
        self.rear_wheel_torque_out.append(self.rear_wheels.wheel_torque)
        self.force_at_wheel_out.append(self.front_wheels.force_at_wheel +
                                self.rear_wheels.force_at_wheel)
        self.front_brake_torque_out.append(self.front_brakes.brake_torque)
        self.rear_brake_torque_out.append(self.rear_brakes.brake_torque)
        self.alpha.append(self.driver.alpha)
        self.beta.append(self.driver.beta)
        self.v_p.append(driver_out['v_p'])
        self.v_i.append(driver_out['v_i'])
        self.v_out.append(vehicle_data['velocity'])
        
        return vehicle_data['velocity']

    def clear_drive_history(self):
        """ Clears the drive history """
        self.v_out = []
        self.power_req_out = []
        self.battery_power_out = []
        self.em_torque_out = []
        self.front_wheel_torque_out = []
        self.rear_wheel_torque_out = []
        self.force_at_wheel_out = []
        self.front_brake_torque_out = []
        self.rear_brake_torque_out = []
        self.alpha = []
        self.beta = []
        self.v_p = []
        self.v_i = []

    def get_velocity_history(self):
        """ Return velocity (in m/s) history of object since last cleared"""
        return self.v_out
