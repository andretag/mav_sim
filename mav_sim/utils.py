import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation

from mav_sim.config import cfg


def quat_from_euler(euler):
    """
        In: euler angles (roll, pitch, yaw) using intrisic/extrinsic convention based on 
        cfg.sim.use_rpy_intrisic_convention
        Out: attitude quaternion in the Pybullet convention (x, y, z, w) -> scalar last
    """
    if cfg.sim.use_rpy_intrisic_convention is False:
        return p.getQuaternionFromEuler(euler)  # r,p,y extrinsic (static)
    # r,p,y intrisic rotation (R = Rz(yaw)*Ry(pitch)*Rx(roll))
    return quat_from_euler_zyx_intr(euler)


def eul_from_quat(quat):
    """
    In: attitude quaternion in the Pybullet convention (x, y, z, w) -> scalar last
    Out: euler angles (roll, pitch, yaw) using intrisic/extrinsic convention based on 
        cfg.sim.use_rpy_intrisic_convention
    """
    if cfg.sim.use_rpy_intrisic_convention is False:
        return p.getEulerFromQuaternion(quat)
    return euler_zyx_intr_from_quat(quat)


def quat_from_euler_zyx_intr(euler_xyz):
    """
        euler xyz is roll, pitch yaw
        assumes euler as ZYX, intrinsics (different from pybullet xyz, extrinsics!)
        Follows default of Matalb functon: 
        function q = eul2quat( eul )

        TODO: switch to scipy
    """
    eul_zyx = np.array([euler_xyz[2], euler_xyz[1], euler_xyz[0]])
    c = np.cos(eul_zyx/2.0)
    s = np.sin(eul_zyx/2.0)
    # Output quat xyzw (different from matlab order, wxyz)
    q = np.array([c[0]*c[1]*s[2]-s[0]*s[1]*c[2],
                  c[0]*s[1]*c[2]+s[0]*c[1]*s[2],
                  s[0]*c[1]*c[2]-c[0]*s[1]*s[2],
                  c[0]*c[1]*c[2]+s[0]*s[1]*s[2]])
    return q


def euler_zyx_intr_from_quat(quat):
    """
        input quat is in format: x, y, z, w (pybullet)
        Returns euler in zyx, intrinsic convention. 
        the angles are ruturned in the order x, y, z (roll pitch yaw), 
        as done in pybullet --- but again, the sequence of rotaitons applied
        differ from the pybullet convention, as is the same as the matlab convention
        (z-y-x intrinsic).

        Follows default of Matalb functon: 
        function q = eul2quat( eul )

        TODO: switch to scipy
    """
    # unit-normalize quat
    quat = quat / np.linalg.norm(quat)

    #
    qx = quat[0]
    qy = quat[1]
    qz = quat[2]
    qw = quat[3]

    eps = 1.0e-11

    aSinInput = -2.0*(qx*qz-qw*qy)
    mask1 = aSinInput >= 1 - 10*eps
    mask2 = aSinInput <= -1 + 10*eps
    if mask1:
        aSinInput = 1.0
    if mask2:
        aSinInput = -1.0

    mask = mask1 or mask2

    eul_zyx = np.array([np.arctan2(2*(qx*qy+qw*qz), qw**2 + qx**2 - qy**2 - qz**2),
                        np.arcsin(aSinInput),
                        np.arctan2(2*(qy*qz+qw*qx), qw**2 - qx**2 - qy**2 + qz**2)])
    if mask:
        eul_zyx[0] = -np.sign(aSinInput) * 2.0 * np.arctan2(qx, qw)
        eul_zyx[2] = 0.0

    eul_xyz = np.array([eul_zyx[2], eul_zyx[1], eul_zyx[0]])
    return eul_xyz  # (same order as pybullet)


def rotm_from_quat(quat):
    """
        In:    
            unit quaternion using the pybullet convention: x, y, z, w as np.array(4,)
        Out: 
            3x3 rotation matrix as np.array(3,3)
    """
    # Note: scipy uses quaternion in scalar last convention, same as pybullet
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_quat.html
    # return Rotation.from_quat(quat).as_matrix()
    R = np.array(p.getMatrixFromQuaternion(quat)).reshape(
            3, 3)
    return R

def rotm_from_euler(euler): 
    q = quat_from_euler(euler)
    R = rotm_from_quat(q)
    return R

def convert_rpy_to_RotM(euler):
    """
    TODO: Andrea: remove
    """
    return rotm_from_euler(euler)
