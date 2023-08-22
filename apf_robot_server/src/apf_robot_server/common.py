import numpy as np


NUM_OBSTACLES = 3
DIST_BTW_OBSTACLES = 2.3
# State Indecies

RS_TARGET = 0
RS_WEIGHTS = RS_TARGET + 2
RS_ROBOT_POSE = RS_WEIGHTS + 2
RS_ROBOT_TWIST = RS_ROBOT_POSE + 3
RS_FORCES = RS_ROBOT_TWIST + 2
RS_COLLISION = RS_FORCES + 3
RS_OBSTACLES = RS_COLLISION + 1
RS_ROSTIME = RS_OBSTACLES + 3 * NUM_OBSTACLES
RS_DETECTED_OBS = RS_ROSTIME + 1
RS_PDGAINS = RS_ROSTIME + 1

# env_state
ENV_TARGET = 0
ENV_ROBOT_TWIST = 2


def _reward(self, rs_state, action):
    reward = 0
    done = False
    info = {}

    # Reward base: Distance to target

    # Calculate distance to the target
    target_coords = np.array([rs_state[RS_TARGET], rs_state[RS_TARGET + 1]])
    robot_coords = np.array([rs_state[RS_ROBOT_POSE], rs_state[RS_ROBOT_POSE + 1]])
    euclidean_dist_2d = np.linalg.norm(target_coords - robot_coords, axis=-1)

    base_reward = -50 * euclidean_dist_2d
    if self.prev_base_reward is not None:
        reward = base_reward - self.prev_base_reward
    self.prev_base_reward = base_reward

    # Negative rewards

    if self.prev_rostime == 0.0:
        self.prev_rostime = rs_state[RS_ROSTIME]
    else:
        # High acceleration
        # 1: Continous Penalty
        # reward = -10 * (abs(rs_state[RS_ROBOT_TWIST] - self.prev_lin_vel) + abs(rs_state[RS_ROBOT_TWIST] - self.prev_ang_vel))

        # 2: Discrete Penalty
        timediff = rs_state[RS_ROSTIME] - self.prev_rostime
        if abs(rs_state[RS_ROBOT_TWIST] - self.prev_lin_vel) / timediff > self.apf_util.get_max_lin_acc():
            reward = reward - 2
        if abs(rs_state[RS_ROBOT_TWIST + 1] - self.prev_ang_vel) / timediff > self.apf_util.get_max_ang_acc():
            reward = reward - 5
        self.prev_rostime = rs_state[RS_ROSTIME]

    self.prev_lin_vel = rs_state[RS_ROBOT_TWIST]
    self.prev_ang_vel = rs_state[RS_ROBOT_TWIST + 1]

    # Long path length (episode length)
    reward = reward - 0.001 * self.elapsed_steps

    # TODO: Distance to obstacle

    # End episode if robot is collides with an object, if it is too close
    # to an object.
    if not self.real_robot:
        if self._sim_robot_collision(rs_state):
            reward = -400.0
            done = True
            info["final_status"] = "collision"
            print("contact_collision")
            print("Distance from target: ", str(euclidean_dist_2d))

        if self._robot_close_to_sim_obstacle(rs_state):
            reward = -400.0
            done = True
            info["final_status"] = "collision"
            print("obs_collision")
            print("Distance from target: ", str(euclidean_dist_2d))

        if self._robot_outside_of_boundary_box(rs_state[RS_ROBOT_POSE : RS_ROBOT_POSE + 3]):
            reward = -200.0
            done = True
            info["final_status"] = "out of boundary"
            print("Robot out of boundary")

    # Target Reached
    if euclidean_dist_2d < self.distance_threshold:
        reward = 800
        done = True
        info["final_status"] = "success"
        print("Target Reached!")

    # Time step exceeded
    if self.elapsed_steps >= self.max_episode_steps:
        done = True
        info["final_status"] = "max_steps_exceeded"
        print("max_step_exceeded")
        print("Distance from target: ", str(euclidean_dist_2d))

    return reward, done, info
