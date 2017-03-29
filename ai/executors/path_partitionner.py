import time

from RULEngine.Util.Pose import Pose
from RULEngine.Util.Position import Position
from RULEngine.Util.geometry import get_distance, conv_position_2_list
from ai.Algorithm.IntelligentModule import Pathfinder
from ai.states.world_state import WorldState
import numpy as np


class Path:
    def __init__(self, start=Position(),  end=Position()):

        self.start = start
        self.goal = end
        self.points = [start, end]

    def join_segments(self, other):
        new_path = Path()
        new_path.points = self.points+other.points[1:]
        new_path.start = self.start
        new_path.goal = other.points[-1]
        return new_path


class PathPartitionner(Pathfinder):
    def __init__(self, p_worldstate: WorldState):
        super().__init__(p_worldstate)
        self.p_worldstate = p_worldstate
        self.game_state = self.p_worldstate.game_state
        self.path = Path(Position(0, 0), Position(0, 0))
        self.res = 100
        self.gap_proxy = 300
        self.max_recurs = 2
        self.pose_obstacle = []

    def fastpathplanner(self, path, depth=0, avoid_dir=None):
        collision_bool = self.is_path_collide(path)
        #print(depth)
        if collision_bool and depth < self.max_recurs:

            [sub_target, avoid_dir] = self.search_point(path, avoid_dir)

            path_1 = Path(path.start, sub_target)

            path_1 = self.fastpathplanner(path_1, depth + 1, avoid_dir)

            path_2 = Path(sub_target, path.goal)
            path_2 = self.fastpathplanner(path_2, depth + 1, avoid_dir)

            path = path_1.join_segments(path_2)

        return path

    def get_path(self, player_id=0, pose_target=Pose()):

        self.path = Path(self.game_state.get_player_pose(player_id).position, pose_target.position)

        for player in self.game_state.game.friends.players.values():
            if player.id == player_id:
                pass
            else:
                self.pose_obstacle += [self.game_state.get_player_pose(player.id).position]


        # for player in self.game_state.game.enemies.players.values():
        #     if player.id == player_id:
        #         pass
        #     else:
        #         self.pose_obstacle += [self.game_state.get_player_pose(player.id).position]

        return self.fastpathplanner(self.path).points[1:]

    def is_path_collide(self, path, pose_obstacle=None):
        collision_bool = 0
        if pose_obstacle is None:
            pose_obstacle = self.pose_obstacle
        if not pose_obstacle:
            return collision_bool
        if get_distance(path.start, path.goal) < 0.001:
            return collision_bool
        direction = np.array(conv_position_2_list(path.goal - path.start)) / get_distance(path.goal, path.start)
        for pose_obs in pose_obstacle:
            vec_robot_2_obs_temp = np.array(conv_position_2_list(pose_obs - path.start))
            len_along_path_temp = np.dot(vec_robot_2_obs_temp, np.transpose(direction))
            dist_from_path_temp = np.sqrt(np.linalg.norm(vec_robot_2_obs_temp)**2 - len_along_path_temp**2)
            if self.gap_proxy > dist_from_path_temp and len_along_path_temp > 0:
                collision_bool = 1
        return collision_bool

    def find_closest_obstacle(self, point, path):

        dist_point_obs = np.inf

        closest_obs = None


        if not self.pose_obstacle:
            return [closest_obs, dist_point_obs]
        if get_distance(path.start, path.goal) < 0.001:
            return [closest_obs, dist_point_obs]
        direction = np.array(conv_position_2_list(point - path.start)) / get_distance(point, path.start)
        for pose_obs in self.pose_obstacle:
            vec_robot_2_obs_temp = np.array(conv_position_2_list(pose_obs - path.start))
            len_along_path_temp = np.dot(vec_robot_2_obs_temp, np.transpose(direction))
            dist_from_path_temp = np.sqrt(np.linalg.norm(vec_robot_2_obs_temp) ** 2 - len_along_path_temp ** 2)
            if self.gap_proxy > dist_from_path_temp and len_along_path_temp > 0:
                dist = get_distance(path.start, pose_obs)
                if dist < dist_point_obs:
                    dist_point_obs = dist
                    closest_obs = pose_obs

        return [closest_obs, dist_point_obs]

    def verify_sub_target(self, sub_target):
        bool_loc = 0
        for pose_obs in self.pose_obstacle:
            dist_sub_2_obs = get_distance(pose_obs, sub_target)
            if dist_sub_2_obs < self.gap_proxy:
                bool_loc = 1
                return bool_loc
        return bool_loc

    def search_point(self, path, avoid_dir=None):
        self.time = time.time()
        pose_robot = path.start
        pose_target = path.goal
        pose_obstacle_closest = self.find_closest_obstacle(pose_target, path)[0]
        if pose_obstacle_closest is None:
            sub_target = pose_target
            return sub_target


        direction = np.array(conv_position_2_list(pose_target - pose_robot)) / get_distance(pose_target, pose_robot)
        vec_robot_2_obs = np.array(conv_position_2_list(pose_obstacle_closest - pose_robot))
        len_along_path = np.dot(vec_robot_2_obs, np.transpose(direction))

        if len_along_path > 0 and len_along_path < get_distance(pose_target, pose_robot):
            vec_perp = np.cross(np.append(direction, 0), np.array([0, 0, 1]))
            vec_perp = vec_perp[0:2]

            if avoid_dir is None:

                sub_target_1 = np.array(conv_position_2_list(pose_robot)) + direction * len_along_path + vec_perp * self.res
                sub_target_2 = np.array(conv_position_2_list(pose_robot)) + direction * len_along_path - vec_perp * self.res
                bool_sub_target_1 = self.verify_sub_target(Position(sub_target_1[0], sub_target_1[1]))
                bool_sub_target_2 = self.verify_sub_target(Position(sub_target_2[0], sub_target_2[1]))

                while bool_sub_target_1:
                    sub_target_1 += vec_perp * self.res
                    bool_sub_target_1 = self.verify_sub_target(Position(sub_target_1[0], sub_target_1[1]))

                sub_target_1 += vec_perp * 0.01 * self.res
                while bool_sub_target_2:

                    sub_target_2 -= vec_perp * self.res
                    bool_sub_target_2 = self.verify_sub_target(Position(sub_target_2[0], sub_target_2[1]))

                sub_target_2 -= vec_perp * 0.01 * self.res

                if abs(get_distance(path.start, Position(sub_target_1[0], sub_target_1[1])) - get_distance(path.start, Position(sub_target_2[0], sub_target_2[1]))) < 600:

                    sub_target = sub_target_1
                    avoid_dir = -vec_perp

                else:
                    if get_distance(path.start, Position(sub_target_1[0], sub_target_1[1])) < get_distance(path.start, Position(sub_target_2[0], sub_target_2[1])):
                        sub_target = sub_target_1
                        avoid_dir = -vec_perp

                    else:
                        sub_target = sub_target_2
                        avoid_dir = vec_perp

            else:
                if np.dot(avoid_dir, np.transpose(vec_perp)) < 0:
                    vec_perp = -vec_perp
                sub_target = np.array(conv_position_2_list(pose_robot)) + direction * len_along_path + vec_perp * self.res

                bool_sub_target = self.verify_sub_target(Position(sub_target[0], sub_target[1]))
                while bool_sub_target:
                    sub_target -= vec_perp * self.res
                    bool_sub_target = self.verify_sub_target(Position(sub_target[0], sub_target[1]))

                sub_target -= vec_perp * 0.01 * self.res
                avoid_dir = vec_perp
            sub_target = Position(sub_target[0], sub_target[1])
        else:
            sub_target = pose_target
        print(time.time() - self.time)
        return [sub_target, avoid_dir]

    def get_next_point(self, robot_id=None):
        pass

    def update(self):
        pass






































