"""Default parameters. Do not change here - use the yaml file instead."""
from yacs.config import CfgNode as CN
import numpy as np

# These are the default parameters. Can be changed by loading a yaml file from
# the mav_sim/params folder
_C = CN()

# Simulation parameters
_C.sim = CN()
_C.sim.gravity = 9.80655                # [m/s^2]
_C.sim.pos_control_step_freq = 10       # [Hz]
_C.sim.attitude_control_step_freq = 100 # [Hz]
_C.sim.physics_freq = 200               # [Hz]
_C.sim.gui = False
_C.sim.use_random_background = False
_C.sim.urdf_model_name = "HX14.urdf"
_C.sim.use_rpy_intrisic_convention = True

# Robot parameters
_C.rbt = CN()

# _C.inertia LOADED FROM URDF!!!
# _C.mass LOADED FROM URDF !!!
_C.rbt.mass = 1.3                               # mav total mass [Kg]
_C.rbt.inertia = [0.0082, 0.0082, 0.0126]       # Diag inertia tensor [ms^2]
_C.rbt.max_thrust_per_g = 1.5                   # [g]
_C.rbt.lin_drag_constant = 0.1                  # [g/s]
_C.rbt.quad_drag_constant = 0.01                # [N/m^2]
_C.rbt.angvel_drag = 0.0001                     # TODO. 10?
_C.rbt.act_delay = 0.00                         # [ms]

# Powertrain and actuators
# [bee Nm] TODO: Andrea: which value is going to be meaningfull? ~1 for octabee
_C.rbt.torque_limit = 10.0
_C.rbt.max_thrust_per_motor = 12.95                  # [N] per motor
_C.rbt.inverse_thrust_curve_ccw = [6.776583, 0.0]    # PWM to thrust, snap_sim
_C.rbt.inverse_thrust_curve_cw = [5.204733, 0.0]     # PWM to thrust, snap_sim
_C.rbt.torque_curve_ccw = [0.135532, 0.0]            # PWM to torque, snap_sim
_C.rbt.torque_curve_cw = [0.104094, 0.0]             # PWM to torque, snap_sim
# thrust force to torque, cd in snap
_C.rbt.prop_thrust_to_torque_coeff = 0.02
_C.rbt.thrust_curve_cw = [0.192133, 0.0]             # thrust to PWM, snap
_C.rbt.thrust_curve_ccw = [0.147567, 0.0]            # thrust to PWM, snap
_C.rbt.n_motors = 6
_C.rbt.motor_spin_direction = [-1, 1, -1, 1, -1, 1]

# Default robot-specific parameters
# Specific to HX14 (e.g., to compute its allocation matrix)
_C.HX14 = CN()
_C.HX14.arm_length = 0.165  # [bee m -> dm]

# Arm length
# Specific to octabee 
_C.octabee = CN()
_C.octabee.lx = 0.22
_C.octabee.ly = 0.12

# Geometric controller parameters
_C.ctr = CN()
# Note: the inertia of the robot is loaded from its URDF file ("robot_urdf"). Set this to match its inertia.
# If true, will feed-forward angular accleration
_C.ctr.open_loop = False
# np.diag(np.array([Ixx, Iyy, Izz]))
_C.ctr.inertia = [0.0082, 0.0082, 0.0126]
_C.ctr.attitude_gains = [1.1, 1.1, 1.1]
_C.ctr.angular_rate_gains = [0.33, 0.33, 0.33]
# Robot mass - set as the one from URDF
_C.ctr.mass = _C.rbt.mass
_C.ctr.gravity = _C.sim.gravity

cfg = _C  # import using `from config import cfg`
