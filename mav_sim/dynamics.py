"""Defines the dynamics of the robot and of the environment."""
import os
import numpy as np
import pybullet as p
from pybullet_utils import bullet_client as bc

from mav_sim.config import cfg
from mav_sim.utils import eul_from_quat, quat_from_euler
from mav_sim.objects_generator import ObstacleFieldGenerator, unit_quat, create_box

ROBOT_URDF_PATH = "assets/robot"
ENV_URDF_PATH = "assets/highbay_single_demonstration"


class MavDynamics:
   

    def __init__(self, seed):
        """Define MAV and simulator properties"""

        # Store attitude controller step frequency
        self.set_att_control_freq(cfg.sim.attitude_control_step_freq)

        # Initialize pybullet and load robot URDF
        connection_mode = p.GUI if cfg.sim.gui else p.DIRECT
        self.ch = bc.BulletClient(connection_mode)
        self.client = self.ch._client
        self.ch.setPhysicsEngineParameter(
            enableFileCaching=False, physicsClientId=self.client)
        self.urdf = os.path.join(os.path.dirname(
            os.path.abspath(__file__)), ROBOT_URDF_PATH, cfg.sim.urdf_model_name)
        print(f"[MavSim] Loading URDF: {self.urdf}.")
        self.mav = self.load_mav_from_urdf(
            W_pos=np.zeros(3), B_rpy=np.zeros(3), W_vel=np.zeros(3))
        self.motor_name_to_index = self.get_motor_link_idxs_from_mav(
            mav_id=self.mav)

        # Load envionrment URDF
        world_urdf_name = "highbay.urdf"
        world_texture_name = "texture_1001.jpg"
        self.world_urdf_pathfile = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), ENV_URDF_PATH, world_urdf_name)
        self.world_texture_pathfile = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), ENV_URDF_PATH, world_texture_name)
        self.obstacles = None
        self.background_plane = None

        # Inialize mav params using properties from URDF
        urdf_mass = self.ch.getDynamicsInfo(
            bodyUniqueId=self.mav, linkIndex=-1, physicsClientId=self.client)[0]
        urdf_inertia = np.array(self.ch.getDynamicsInfo(
            bodyUniqueId=self.mav, linkIndex=-1, physicsClientId=self.client)[2])
        assert cfg.rbt.mass == urdf_mass, "Mass in URDF and in .yaml file differ!"
        assert np.allclose(np.array(
            cfg.rbt.inertia), urdf_inertia), "Inertia in URDF and in .yaml file differ!"

        np.random.seed(seed)

        # External force disturbances
        self.W_ext_f = np.zeros(3)      # [N] External disturbance

        # Reset state of pybullet and MAV.
        self.reset_dynamics()

    def get_motor_link_idxs_from_mav(self, mav_id):
        # https://github.com/bulletphysics/bullet3/pull/1082
        # " In bullet linkIndex == jointIndex"
        motor_name_to_index = {}
        for id in range(self.ch.getNumJoints(mav_id)):
            link_name = self.ch.getJointInfo(mav_id, id)[12].decode('UTF-8')
            if "motor" in link_name:
                motor_name_to_index[link_name] = id
        return motor_name_to_index

    def reset_dynamics(self):
        """Initiailze (or re-initialize) Pybullet and MAV state"""

        # Reset MAV and world state
        self.reset_to_init_state()

    def step_dynamics(self, B_motor_forces_z: np.ndarray, B_motor_torques_z: np.ndarray):
        """Update the dynamics of the MAV given motor forces"""
        assert B_motor_forces_z.shape == (len(self.motor_name_to_index), )
        assert B_motor_torques_z.shape == (len(self.motor_name_to_index), )

        self.ch.setTimeStep(1.0/cfg.sim.physics_freq,
                            physicsClientId=self.client)

        for _ in range(int(cfg.sim.physics_freq/cfg.sim.attitude_control_step_freq)):

            # Thrust force and torque per motor
            # TODO: this assumes that the motor in URDF are in same order as allocation matrix
            for i in list(self.motor_name_to_index.values()):

                motor_link_idx = i  # Big assumption: motors link indexes
                # are always 0...5 for motor 1...6.
                B_thrust_f_motor_i = [0.0, 0.0, B_motor_forces_z[i]]
                B_drag_torque_motor_i = [0.0, 0.0, B_motor_torques_z[i]]

                self.ch.applyExternalForce(objectUniqueId=self.mav, linkIndex=motor_link_idx,
                                           forceObj=B_thrust_f_motor_i, posObj=list(
                                               np.zeros(3)),
                                           flags=self.ch.LINK_FRAME, physicsClientId=self.client)

                self.ch.applyExternalTorque(objectUniqueId=self.mav, linkIndex=motor_link_idx,
                                            torqueObj=B_drag_torque_motor_i,
                                            flags=self.ch.LINK_FRAME, physicsClientId=self.client)

            # External disturbance in COM
            B_f_ext = self.transform_vect_from_W_to_B(self.W_ext_f)
            # There must be a bug with pybullet and the force cannot be fed in the inertial frame....
            self.ch.applyExternalForce(objectUniqueId=self.mav, linkIndex=-1, forceObj=list(B_f_ext), posObj=list(
                np.zeros(3)), flags=self.ch.LINK_FRAME, physicsClientId=self.client)

            # Body drag force/torque in COM
            # Get velocity in body frame
            B_vel, B_omega = self.get_B_velocities()
            B_drag_f = self.get_B_drag_f(B_vel)
            self.ch.applyExternalForce(objectUniqueId=self.mav, linkIndex=-1, forceObj=list(B_drag_f), posObj=list(
                np.zeros(3)), flags=self.ch.LINK_FRAME, physicsClientId=self.client)

            B_drag_torque = - cfg.rbt.angvel_drag*B_omega
            self.ch.applyExternalTorque(objectUniqueId=self.mav, linkIndex=-1, torqueObj=list(B_drag_torque),
                                        flags=self.ch.LINK_FRAME, physicsClientId=self.client)

            # Step physics.
            self.ch.stepSimulation(physicsClientId=self.client)
            #print("Stepped physics stepSimulation")

            # Update estimate of acceleration. May need low pass filter.
            s = self.get_state()
            W_vel = s["W_vel"]
            self.W_acc = (W_vel - self.W_vel_prev)*float(cfg.sim.physics_freq)
            self.W_vel_prev = W_vel

    def get_B_drag_f(self, B_vel: np.ndarray):
        """
        Computes drag force in body frame according to drag model
        """
        B_vel_mag = np.linalg.norm(B_vel)
        B_vel_dir = np.zeros(3)
        if B_vel_mag <= 1e-4:
            return np.zeros(3)
        B_vel_dir = B_vel/B_vel_mag
        c1 = cfg.rbt.lin_drag_constant
        c2 = cfg.rbt.quad_drag_constant
        B_f_drag = -(c1*B_vel_mag + c2*B_vel_mag**2)*B_vel_dir
        return B_f_drag

    def transform_vect_from_W_to_B(self, W_v):
        _, W_quat_B = self.ch.getBasePositionAndOrientation(
            self.mav, physicsClientId=self.client)
        W_rot_B = np.array(
            self.ch.getMatrixFromQuaternion(W_quat_B)).reshape(3, 3)
        B_v = np.matmul(W_rot_B.T, W_v)
        return B_v

    def reset_to_init_state(self, W_pos=np.zeros(3), B_rpy=np.zeros(3), W_vel=np.zeros(3)):
        """Reinitialize MAV state to a desired value"""
        assert W_pos.shape == (3,)
        assert B_rpy.shape == (3,)
        assert W_vel.shape == (3,)

        # Pybullet settings
        self.ch.resetSimulation(physicsClientId=self.client)
        self.ch.setGravity(0.0, 0.0, -cfg.sim.gravity,
                           physicsClientId=self.client)
        self.ch.setRealTimeSimulation(0, physicsClientId=self.client)
        self.ch.setTimeStep(1.0/cfg.sim.physics_freq,
                            physicsClientId=self.client)

        # Re-load mav model from URDF and its state.
        self.mav = self.load_mav_from_urdf(
            W_pos=W_pos, B_rpy=B_rpy, W_vel=W_vel)
        self.motor_name_to_index = self.get_motor_link_idxs_from_mav(
            mav_id=self.mav)
        self.world = self.load_world_from_urdf(urdf_path_filename=self.world_urdf_pathfile,
                                               texture_path_filename=self.world_texture_pathfile)
        self.W_ext_f = np.zeros(3)

        # Acceleration in body frame computed when stepping dynamics.
        self.W_acc = np.zeros(3)        # Acceleration in Inertial frame
        self.W_vel_prev = self.get_state()["W_vel"]

        # Add obstacles:
        self.obstacles = self.add_obstacles(poisson_intensity=0.0)

        # Add background plane of random color
        if cfg.sim.use_random_background:
            print("Loading random background plane")
            self.background_plane = self.get_colored_background_plane()

    def set_att_control_freq(self, att_control_step_freq_hz):
        assert att_control_step_freq_hz > 0
        assert cfg.sim.physics_freq >= att_control_step_freq_hz
        # Fraquency at which step_dynamics is called.
        self.attitude_control_step_freq = att_control_step_freq_hz

    def add_obstacles(self, poisson_intensity=0.5, random=False):
        """
        Add a set of N obstacles to the environment
        w or w/o randomized position. 
        Useful related code: 
        """
        return ObstacleFieldGenerator(poisson_intensity=poisson_intensity).get_obstacles(client_handle=self.ch)

    def get_colored_background_plane(self):
        rgb_color = np.random.uniform(low=0.0, high=1.0, size=(3,)).tolist()
        position = [0.0, 0.0, -2.0]
        pose = (position, unit_quat())
        alpha_channel = [1.]
        rgba_color = rgb_color + alpha_channel
        XY_SIZE = 100
        DEPTH = 0.1
        print(f"Plane colors: {rgb_color}")
        return create_box(half_extents=[XY_SIZE, XY_SIZE, DEPTH], client_handle=self.ch, pose=pose, color=rgba_color)

    def get_state(self):
        """
        Returns a dictionary containing the current state of the MAV. 
        """
        #
        # https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html
        # getBasePositionAndOrientation reports the current position
        # and orientation of the base (or root link) of the body in
        # Cartesian world coordinates.
        # Quaternion convention:
        # The orientation is a quaternion in [x,y,z,w] format.
        s = {}
        s["W_pos"], s["W_quat_B"] = self.ch.getBasePositionAndOrientation(
            self.mav, physicsClientId=self.client)  # Quatrnion in scalar-last convention (used by pybullet)
        # Quaternion in scalar-first convention
        s["W_quat_B_wxyz"] = np.concatenate(
            (s["W_quat_B"][3:4], s["W_quat_B"][0:3]))
        s["B_rpy"] = eul_from_quat(s["W_quat_B"])
        W_vel, W_omega = self.ch.getBaseVelocity(
            self.mav, physicsClientId=self.client)
        s["W_vel"] = np.array(W_vel)
        s["W_omega"] = np.array(W_omega)
        s["W_rot_B"] = np.array(
            self.ch.getMatrixFromQuaternion(s["W_quat_B"])).reshape(3, 3)
        s["B_vel"] = np.matmul(s["W_rot_B"].T, s["W_vel"])
        s["B_omega"] = np.matmul(s["W_rot_B"].T, s["W_omega"])
        return s

    def get_yaw(self):
        _, W_quat_B = self.ch.getBasePositionAndOrientation(
            self.mav, physicsClientId=self.client)
        B_rpy = eul_from_quat(W_quat_B)
        return B_rpy[2]

    def get_B_velocities(self):
        """Returns the velocity of the base in body frame"""
        s = self.get_state()
        return s["B_vel"], s["B_omega"]

    def set_ext_force(self, W_ext_f):
        """Set external disturbance force"""
        assert W_ext_f.shape == (3,)
        self.W_ext_f = W_ext_f

    def update_drag_params(self, lin_drag_cx, quad_drag_cx):
        """Update drag coefficients for isotropic drag model"""
        assert lin_drag_cx >= 0.
        assert quad_drag_cx >= 0.
        cfg.lin_drag_constant = lin_drag_cx
        cfg.quad_drag_constant = quad_drag_cx

    def load_mav_from_urdf(self, W_pos: np.ndarray, B_rpy: np.ndarray, W_vel: np.ndarray):
        """
            Load MAV model from URDF 
            and overrides some (unrealistic) 
            default properties of pybullet
        """
        assert W_pos.shape == (3,)
        assert B_rpy.shape == (3,)
        assert W_vel.shape == (3,)
        mav_object_id = self.ch.loadURDF(os.path.join(ROBOT_URDF_PATH, self.urdf),
                                         basePosition=W_pos,
                                         baseOrientation=quat_from_euler(
                                             B_rpy),
                                         flags=self.ch.URDF_USE_INERTIA_FROM_FILE,  # self.ch.URDF_MERGE_FIXED_LINKS |
                                         physicsClientId=self.client,
                                         useFixedBase=False)
        assert mav_object_id >= 0, f"Unable to load URDF file at {self.urdf}. "

        # Reset mav velocity
        self.ch.resetBaseVelocity(objectUniqueId=mav_object_id,
                                  linearVelocity=W_vel,
                                  physicsClientId=self.client)
        # Set damping coefficients used to stabilize pybullet ODEs to a more realistic
        # value for a UAV.
        # See "changeDynamics" in: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#
        #print(self.ch.getDynamicsInfo(bodyUniqueId=self.mav, linkIndex=-1))
        #print(self.ch.getDynamicsInfo(bodyUniqueId=self.mav, linkIndex=0))
        for id in range(self.ch.getNumJoints(mav_object_id)):
            self.ch.changeDynamics(bodyUniqueId=mav_object_id, linkIndex=id,
                                   linearDamping=0, angularDamping=0, physicsClientId=self.client)

        # TODO: Andrea: remove
        # Manually set inertia on z axis since loadURDF will not allow to set some values
        # (see https://pybullet.org/Bullet/phpBB3/viewtopic.php?f=9&t=13248&p=43523#p43523)
        #self.ch.changeDynamics(bodyUniqueId=mav_object_id, linkIndex=0, localInertiaDiagonal=[0.03, 0.03, 0.09], physicsClientId=self.client)
        #self.ch.changeDynamics(bodyUniqueId=mav_object_id, linkIndex=1, localInertiaDiagonal=[0.0, 0.0, 0.0], physicsClientId=self.client)
        return mav_object_id

    def load_world_from_urdf(self, urdf_path_filename, texture_path_filename):

        # Load urdf - this will load a mesh, but w/o texture
        world_object_id = self.ch.loadURDF(urdf_path_filename,
                                           flags=self.ch.URDF_USE_INERTIA_FROM_FILE,  # self.ch.URDF_MERGE_FIXED_LINKS |
                                           physicsClientId=self.client,
                                           useFixedBase=True)
        # Load texture
        texture_id = self.ch.loadTexture(texture_path_filename)
        self.ch.changeVisualShape(world_object_id, -1,
                                  textureUniqueId=texture_id,
                                  physicsClientId=self.client)
        return world_object_id

    def get_onboard_camera_observation(self):
        """
        Returns a camera observation from the current pose of the robot
        """
        W_pos, W_quat_B = self.ch.getBasePositionAndOrientation(
            self.mav, physicsClientId=self.client)
        return self.get_onboard_camera_observation_at_pose(W_pos, W_quat_B)

    def get_onboard_camera_observation_at_position_roll_pitch(self, pos, rp):
        """
        Returns the corresponding observation at a given position, orientation
        Inputs: 
            pos: position of the camera (in meters)/NOT normalized
            rp: orientation (rad)/not normalized. Yaw will be set to the current yaw of the robot.
        """
        # Get current yaw
        yaw = self.get_yaw()
        B_rpy = np.array([rp[0], rp[1], yaw])
        W_quad_B_desired = quat_from_euler(B_rpy)
        return self.get_onboard_camera_observation_at_pose(W_pos=pos, W_quat_B=W_quad_B_desired)

    def get_onboard_camera_observation_at_pose(self, W_pos, W_quat_B):
        """
            Simulates a monocular/depth camera attached to the 
            belly of a robot at the given pose (W_pos, W_quat_B), 
            and returns the corresponding RGB/depth image.
            TODO: Andrea: load extrinsics, intrisics
            and image size from yaml file. 
            All the outputs are in the 0-255 range. 
        """
        # Attitude rotation matrix
        W_rot_B = np.array(
            self.ch.getMatrixFromQuaternion(W_quat_B)).reshape(3, 3)

        B_R_C = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]]
                         )    # Downward facing camera
        # B_R_C = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])     # Forward facing camera

        W_T_C = np.zeros((4, 4))
        W_T_C[-1, -1] = 1.0
        W_T_C[0:3, 0:3] = np.matmul(W_rot_B, B_R_C)
        W_T_C[0, 3] = W_pos[0]
        W_T_C[1, 3] = W_pos[1]
        W_T_C[2, 3] = W_pos[2]
        viewMatrix = list(W_T_C)
        target_vector = np.matmul(W_T_C[0:3, 0:3], np.array([0, 0, 1]))
        up_vector = np.matmul(W_T_C[0:3, 0:3], np.array([0, -1, 0]))

        viewMatrix = self.ch.computeViewMatrix(cameraEyePosition=list(W_pos),
                                               cameraTargetPosition=list(
                                                   100*target_vector + W_pos),
                                               cameraUpVector=[1, 0, 0],
                                               physicsClientId=self.client)

        projectionMatrix = self.ch.computeProjectionMatrixFOV(
            fov=100.0,
            aspect=1.0,
            nearVal=0.01,
            farVal=10.0,
            physicsClientId=self.client)

        # TODO: load params from yaml file
        w, h, rgba, depth, mask = self.ch.getCameraImage(width=64, height=64,
                                                         viewMatrix=viewMatrix,
                                                         projectionMatrix=projectionMatrix,
                                                         physicsClientId=self.client)
        # More ideas on how to control the camera can be found here
        #   https://github.com/benelot/pybullet-gym/blob/ec9e87459dd76d92fe3e59ee4417e5a665504f62/pybulletgym/envs/mujoco/envs/env_bases.py
        rgb_array = np.array(rgba)  # "a" stands for alpha channel
        rgb_array = rgb_array[:, :, :3]
        # Image shape needed for pytorch:
        #   https://discuss.pytorch.org/t/dimensions-of-an-input-image/19439
        #   [channels, height, width]
        # https://stackoverflow.com/questions/43829711/what-is-the-correct-way-to-change-image-channel-ordering-between-channels-first
        rgb_array = np.moveaxis(rgb_array, -1, 0)
        # Take average across 0-th dimension to obtain greyscale image
        greyscale_array = np.mean(rgb_array, (0))
        # output a 3D array (1, height, width)
        greyscale_array = np.expand_dims(greyscale_array, axis=(0,))
        return greyscale_array, depth

    def get_B_g(self):
        """
            Returns gravity in body frame B_g. 
        """
        s = self.get_state()
        W_g = np.array([0, 0, -cfg.sim.gravity])
        B_g = np.matmul(s["W_rot_B"].T, W_g)
        return B_g