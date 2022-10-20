"""Allocation (or Mixer) matrix mapping body wrenches to actuator forces."""
import numpy as np

from mav_sim.config import cfg


def compute_allocation_matrix_HX14():
    """
        Maps body thrust and torques to motor commands
    """
    inverse_allocation = compute_inverse_allocation_matrix_HX14()
    return np.linalg.pinv(inverse_allocation)

def compute_inverse_allocation_matrix_HX14():
    """
        Maps motor commands to body thrust and torques
    """
    a = 2.0*np.pi/6.0
    l = cfg.HX14.arm_length
    lsa2 = l*np.cos(a/2)
    lca2 = l*np.cos(a/2.)
    cd = cfg.rbt.prop_thrust_to_torque_coeff
    rx = 0.0
    ry = 0.0

    # Compute inverse allocation matrix: maps motor force to wrench.
    inverse_allocation = np.array([[1., 1., 1., 1., 1., 1.],
                                   [lsa2-ry, l - ry,    lsa2 - ry,  -
                                       lsa2 - ry, -l - ry, -lsa2 - ry],
                                   [-lca2 + rx,  0. + rx,   lca2 + rx,
                                       lca2 + rx,  0. + rx,  -lca2 + rx],
                                   [cd,   -cd,   cd,    -cd,    cd,  -cd]
                                   ])
    return inverse_allocation


def compute_allocation_matrix_octabee():
    """
        Maps body thrust and torques to motor commands
    """
    inverse_allocation = compute_inverse_allocation_matrix_octabee()
    return np.linalg.pinv(inverse_allocation)


def compute_inverse_allocation_matrix_octabee():
    """
        Maps motor commands to body thrust and torques
    """
    cd = cfg.rbt.prop_thrust_to_torque_coeff
    lx = cfg.octabee.lx
    ly = cfg.octabee.ly
    inverse_allocation = np.array([[1., 1., 1., 1., ],
                                   [-ly, ly, ly, -ly],
                                   [-lx, -lx, lx, lx],
                                   [cd, -cd, cd, -cd]])
    return inverse_allocation


class MotorAllocation():
    """
            Motor thrust allocation/mixer matrix.
            Provides way to convert desired body thrust/torques 
            to motor thrust commands
    """
    allocation_matrix: np.ndarray

    def __init__(self):
        if cfg.sim.urdf_model_name == "octabee.urdf":
            self.allocation_matrix = compute_allocation_matrix_octabee()
            print("Using octabee allocation matrix")
        elif cfg.sim.urdf_model_name == "HX14.urdf":
            self.allocation_matrix = compute_allocation_matrix_HX14()
            print("Using HX14 allocation matrix")
        else:
            raise NotImplementedError(
                f"Unable to find allocation matrix associated to {cfg.sim.urdf_model_name}.")

    def compute_desired_motor_forces(self, B_thrust_f: np.ndarray, B_torque: np.ndarray):
        """
                Input: desired body thrust, torque
                Output: normalized PWM commanded to each motor
        """
        assert B_thrust_f.shape == (3,)
        assert B_torque.shape == (3,)

        # Saturate thrust and torque.
        max_thrust_f = cfg.rbt.max_thrust_per_g*cfg.rbt.mass*cfg.sim.gravity
        #assert np.all(B_thrust_f < max_thrust_f), f"Saturating thrust: {B_thrust_f}"
        #assert np.all(B_torque < cfg.rbt.torque_limit), "Saturating torque."
        B_thrust_f = np.clip(a=B_thrust_f, a_min=np.zeros(
            3), a_max=np.ones(3)*max_thrust_f)
        B_torque = np.clip(a=B_torque, a_min=-np.ones(3) *
                           cfg.rbt.torque_limit, a_max=np.ones(3)*cfg.rbt.torque_limit)

        # Apply motor allocation.
        body_force_torques = np.array(
            [B_thrust_f[2], B_torque[0], B_torque[1], B_torque[2]])
        desired_motors_force = np.matmul(
            self.allocation_matrix, body_force_torques)
        return desired_motors_force
