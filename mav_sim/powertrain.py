"""
    Methods to simulate the powertrain of an UAV:
    - thrust to actuator commands mappings 
    - actuation delays
"""
import numpy as np
import collections

from mav_sim.config import cfg


class Buffer():
    """
        Circular Buffer of size maxlen. 
        Most recent data in the back.
    """

    def __init__(self, maxlen):
        self.initialized = False
        self.queue = collections.deque(maxlen=maxlen)

    def add(self, data):
        if not self.initialized:
            # Fill buffer with current data.
            for _ in range(self.queue.maxlen):
                self.queue.append(data)
            self.initialized = True
        else:
            self.queue.append(data)

    def reset(self):
        """
            Empty the buffer.
        """
        if self.initialized is True:
            max_queue_len = self.queue.maxlen
        else:
            # Just for safety...
            max_queue_len = len(self.queue)
        for _ in range(max_queue_len):
            self.queue.pop()
        self.initialized = False


class PowertrainModel():
    """
        Model of quadcopter powertrain
    """

    def __init__(self):
        assert len(cfg.rbt.thrust_curve_cw) == 2
        assert len(cfg.rbt.thrust_curve_ccw) == 2
        assert len(cfg.rbt.inverse_thrust_curve_cw) == 2
        assert len(cfg.rbt.inverse_thrust_curve_ccw) == 2
        assert len(cfg.rbt.torque_curve_cw) == 2
        assert len(cfg.rbt.torque_curve_ccw) == 2
        assert len(cfg.rbt.motor_spin_direction) == cfg.rbt.n_motors
        self.delay_act_cmds = Buffer(maxlen=max(
            1, int(cfg.rbt.act_delay*cfg.sim.attitude_control_step_freq)))
        self.reset()

    def compute_motor_forces_torques(self, desired_motor_forces):
        """
            Computes the acutal force produced  by the actuators, 
            given desired forces, using the
            voltages to thrust mapping and its (identified) inverse model, 
            which may be slightly different from the correct one. 
        """
        assert desired_motor_forces.shape == (cfg.rbt.n_motors,)
        desired_motor_forces = np.clip(a=desired_motor_forces, a_min=np.zeros(cfg.rbt.n_motors),
                                       a_max=np.ones(cfg.rbt.n_motors)*cfg.rbt.max_thrust_per_motor)
        motor_forces = np.zeros(cfg.rbt.n_motors)
        motor_torques = np.zeros(cfg.rbt.n_motors)
        for motor_id in range(cfg.rbt.n_motors):
            if cfg.rbt.motor_spin_direction[motor_id] < 0:  # cw
                thrust_curve = cfg.rbt.thrust_curve_cw
                inverse_thrust_curve = cfg.rbt.inverse_thrust_curve_cw
                torque_curve = cfg.rbt.torque_curve_cw
            else:
                thrust_curve = cfg.rbt.thrust_curve_ccw
                inverse_thrust_curve = cfg.rbt.inverse_thrust_curve_ccw
                torque_curve = cfg.rbt.torque_curve_ccw
            pwm = thrust_curve[0]*desired_motor_forces[motor_id]
            motor_forces[motor_id] = inverse_thrust_curve[0]*pwm
            motor_torques[motor_id] = - \
                cfg.rbt.motor_spin_direction[motor_id]*torque_curve[0]*pwm
        return motor_forces, motor_torques

    def compute_ideal_motor_forces_torques(self, desired_motor_forces):
        """
        pass-through. 
        """
        desired_motor_forces = np.clip(a=desired_motor_forces, a_min=np.zeros(cfg.rbt.n_motors),
                                       a_max=np.ones(cfg.rbt.n_motors)*cfg.rbt.max_thrust_per_motor)
        motor_forces = np.zeros(cfg.rbt.n_motors)
        motor_torques = np.zeros(cfg.rbt.n_motors)
        for motor_id in range(cfg.rbt.n_motors):
            motor_forces[motor_id] = desired_motor_forces[motor_id]
            motor_torques[motor_id] = -cfg.rbt.motor_spin_direction[motor_id] * \
                desired_motor_forces[motor_id]
        return motor_forces, motor_torques

    def compute_ideal_motor_forces_torques_with_delay(self, desired_motor_forces):
        """
        pass-through w/ actuation delay. 
        """
        desired_motor_forces = np.clip(a=desired_motor_forces, a_min=np.zeros(cfg.rbt.n_motors),
                                       a_max=np.ones(cfg.rbt.n_motors)*cfg.rbt.max_thrust_per_motor)
        motor_forces = np.zeros(cfg.rbt.n_motors)
        motor_torques = np.zeros(cfg.rbt.n_motors)
        for motor_id in range(cfg.rbt.n_motors):
            motor_forces[motor_id] = desired_motor_forces[motor_id]
            motor_torques[motor_id] = -cfg.rbt.motor_spin_direction[motor_id] * \
                desired_motor_forces[motor_id]
        self.delay_act_cmds.add((motor_forces, motor_torques))
        return self.delay_act_cmds.queue[0][0], self.delay_act_cmds.queue[0][1]

    def reset(self):
        self.delay_act_cmds.reset()
        init_motor_force = np.ones(cfg.rbt.n_motors) * \
            cfg.rbt.mass*cfg.sim.gravity/cfg.rbt.n_motors
        init_motor_torque = np.zeros(cfg.rbt.n_motors)
        # init with hover motor forces?
        self.delay_act_cmds.add((init_motor_force, init_motor_torque))
