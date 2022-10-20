"""
    Tools to generate an obstacle field, and other 3D objects. 
    The following methods have been adapted from here: 
    https://github.com/caelan/pybullet-planning/blob/fd8a36d903fe83666a27124d331ea31af4788a5d/pybullet_tools/utils.py#L2494
"""
import numpy as np
import pybullet as p
from collections import namedtuple

from mav_sim.utils import quat_from_euler


STATIC_MASS = 0.0

RGBA = namedtuple('RGBA', ['red', 'green', 'blue', 'alpha'])
BLUE = RGBA(0, 0, 1, 1)
RED = RGBA(1, 0, 0, 1)
NULL_ID = -1


def unit_point():
    return (0., 0., 0.)


def unit_quat():
    return quat_from_euler([0, 0, 0])  # [X,Y,Z,W]


def unit_pose():
    return (unit_point(), unit_quat())


def create_collision_shape(geometry, client_handle, pose=unit_pose()):
    # TODO: removeCollisionShape
    # https://github.com/bulletphysics/bullet3/blob/5ae9a15ecac7bc7e71f1ec1b544a55135d7d7e32/examples/pybullet/examples/getClosestPoints.py
    point, quat = pose
    collision_args = {
        'collisionFramePosition': point,
        'collisionFrameOrientation': quat,
        'physicsClientId': client_handle._client,
        # 'flags': p.GEOM_FORCE_CONCAVE_TRIMESH,
    }
    collision_args.update(geometry)
    if 'length' in collision_args:
        # TODO: pybullet bug visual => length, collision => height
        collision_args['height'] = collision_args['length']
        del collision_args['length']
    return client_handle.createCollisionShape(**collision_args)


def create_visual_shape(geometry, client_handle, pose=unit_pose(), color=BLUE, specular=None):
    if (color is None):  # or not has_gui():
        return NULL_ID
    point, quat = pose
    visual_args = {
        'rgbaColor': color,
        'visualFramePosition': point,
        'visualFrameOrientation': quat,
        'physicsClientId': client_handle._client,
    }
    visual_args.update(geometry)
    if specular is not None:
        visual_args['specularColor'] = specular
    return client_handle.createVisualShape(**visual_args)


def create_body(client_handle, collision_id=NULL_ID, visual_id=NULL_ID, mass=STATIC_MASS):
    return client_handle.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                                         baseVisualShapeIndex=visual_id, physicsClientId=client_handle._client)


def create_shape(geometry, client_handle, pose=unit_pose(), collision=True, **kwargs):
    collision_id = create_collision_shape(
        geometry, client_handle=client_handle, pose=pose) if collision else NULL_ID
    visual_id = create_visual_shape(
        geometry, client_handle=client_handle, pose=pose, **kwargs)  # if collision else NULL_ID
    return collision_id, visual_id


def create_cylinder(radius, height, client_handle, mass=STATIC_MASS, color=BLUE, **kwargs):
    collision_id, visual_id = create_shape(get_cylinder_geometry(
        radius, height), client_handle=client_handle, color=color, **kwargs)
    return create_body(client_handle=client_handle, collision_id=collision_id, visual_id=visual_id, mass=mass)


def create_plane(normal, client_handle, mass=STATIC_MASS, color=RED, **kwargs):
    collision_id, visual_id = create_shape(get_plane_geometry(
        normal), client_handle=client_handle, color=color, **kwargs)
    return create_body(client_handle=client_handle, collision_id=collision_id, visual_id=visual_id, mass=mass)


def create_box(half_extents, client_handle, mass=STATIC_MASS, color=RED, **kwargs):
    collision_id, visual_id = create_shape(get_box_geometry(
        half_extents), client_handle=client_handle, color=color, **kwargs)
    return create_body(client_handle=client_handle, collision_id=collision_id, visual_id=visual_id, mass=mass)


def get_cylinder_geometry(radius, height):
    return {
        'shapeType': p.GEOM_CYLINDER,
        'radius': radius,
        'length': height,
    }


def get_plane_geometry(normal):
    return {
        'shapeType': p.GEOM_PLANE,
        'planeNormal': normal
    }


def get_box_geometry(half_extents):
    return {
        'shapeType': p.GEOM_BOX,
        'halfExtents': half_extents
    }


class ObstacleFieldGenerator():

    def __init__(self, poisson_intensity=0.5, x_min=-2, y_min=1, x_max=4, y_max=7, r_min=0.1, r_max=0.5, h_min=1.5, h_max=3.0):
        """
        Randomly positions N_obstacles cylindrical obstacles inside a rectangular area. 
        """
        self.poisson_intensity = poisson_intensity
        # Obstacles are generated inside rectangle
        self.x_min = x_min
        self.y_min = y_min
        self.x_max = x_max
        self.y_max = y_max

        # Properties of the cylindrical obstacles:
        # Radius
        self.r_min = r_min
        self.r_max = r_max
        # Height
        self.h_min = h_min
        self.h_max = h_max

    def get_obstacles(self, client_handle):
        # http://connor-johnson.com/2014/02/25/spatial-point-processes/
        # https://stackoverflow.com/questions/31133232/poisson-point-process-in-python-3-with-numpy-without-scipy
        area = (self.x_max - self.x_min)*(self.y_max - self.y_min)
        N_obstacles = np.random.poisson(area*self.poisson_intensity)
        positions = np.random.uniform(low=(self.x_min, self.y_min),
                                      high=(self.x_max, self.y_max),
                                      size=(N_obstacles, 2))
        radiuses = np.random.uniform(
            low=self.r_min, high=self.r_max, size=N_obstacles)
        heights = np.random.uniform(
            low=self.h_min, high=self.h_max, size=N_obstacles)

        return [create_cylinder(radius=r, height=h, client_handle=client_handle, pose=((pos[0], pos[1], -1.0 + h/2.0), unit_quat)) for (r, h, pos) in zip(radiuses, heights, positions)]
