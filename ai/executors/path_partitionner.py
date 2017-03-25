from RULEngine.Util.Position import Position
from RULEngine.Util.geometry import get_distance, conv_position_2_list
from ai.Algorithm.IntelligentModule import Pathfinder
from ai.states.world_state import WorldState
import numpy as np


class Path:
    def __init__(self, start = Position(),  end = Position()):

        self.start = start
        self.goal = end
        self.points = [start,end]

    def join_segments(self,other):
        self.points = self.points.append(other.points[1:])
        self.start = self.start
        self.goal = self.points[-1]


class PathPartitionner(Pathfinder):
    def __init__(self, p_worldstate: WorldState):
        super().__init__(p_worldstate)
        self.p_worldstate = p_worldstate
        self.game_state = self.p_worldstate.game_state
        self.path = Path(Position(),Position())
        self.res = 20
        self.gap_proxy = 400
        self.max_recurs = 4
        self.pose_obstacle = []

    def init_path(self, player_id, pose_target):
        self.path.start = self.game_state.get_player_pose(player_id).position
        self.path.goal = pose_target.position
        self.path.points = [self.path.start,self.path.goal]
        for player in self.game_state.game.friends.players:
            if player.id == player_id:
                pass
            else:
                self.pose_obstacle.append(self.game_state.get_player_pose(player.id).position)
        for player in self.game_state.game.enemies.players:
            if player.id == player_id:
                pass
            else:
                self.pose_obstacle.append(self.game_state.get_player_pose(player.id).position)

    def is_path_collide (self,path, pose_obstacle = None):
        collision_bool = 0
        if pose_obstacle is None:
            pose_obstacle = self.pose_obstacle
        if not pose_obstacle:
            return collision_bool
        if get_distance(path.start,path.goal)  < 0.001:
            return collision_bool
        direction = np.array(conv_position_2_list(path.goal - path.start) / get_distance(path.goal, path.start))
        for pose_obs in pose_obstacle:
            vec_robot_2_obs_temp = np.array(conv_position_2_list(pose_obs - path.start))
            len_along_path_temp = vec_robot_2_obs_temp * np.transpose(direction)
            dist_from_path_temp = np.sqrt(np.linalg.norm(vec_robot_2_obs_temp) ^ 2 - len_along_path_temp ^ 2)
            if self.gap_proxy > dist_from_path_temp & len_along_path_temp > 0:
                collision_bool = 1
                return collision_bool

    def find_closest_obstacle(self, point, path,):
        pose_obstacle_col = []
        for pose_obs in self.pose_obstacle:
            if self.is_path_collide(Path(path.start, point), pose_obs):
                pose_obstacle_col.append(pose_obs)
        if not pose_obstacle_col:
            closest_obs = np.math.nan
            dist_point_obs = np.math.nan
            return [closest_obs, dist_point_obs]
        dist_point_obs = get_distance(path.start, pose_obstacle_col[0])
        closest_obs = pose_obstacle_col[1]
        for pose_obs_col in pose_obstacle_col:
            if get_distance(path.start, pose_obs_col) < dist_point_obs:
                dist_point_obs = get_distance(path.start, pose_obs_col)
                closest_obs = pose_obs_col
        return [closest_obs, dist_point_obs]

    def verify_sub_target(self, sub_target):
        bool = 0
        for pose_obs in self.pose_obstacle:
            dist_sub_2_obs = get_distance(pose_obs, sub_target)
            if dist_sub_2_obs < self.gap_proxy:
                bool = 1
                return bool
        return bool

    def search_point(self,path, avoid_dir = np.nan):
        pose_robot = path.start
        pose_target = path.goal
        pose_obstacle_closest = self.find_closest_obstacle(pose_target, path)[0]
        if np.isnan(pose_obstacle_closest):
            sub_target = pose_target
            return sub_target

        direction = np.array(conv_position_2_list(pose_target - pose_robot) / get_distance(pose_target, pose_robot))
        vec_robot_2_obs = np.array(conv_position_2_list(pose_obstacle_closest - pose_robot))
        len_along_path = vec_robot_2_obs * np.transpose(direction)

        if len_along_path > 0 & len_along_path < get_distance(pose_target - pose_robot):
            vec_perp = np.cross([np.append(direction, 0)], [0, 0, 1])
            vec_perp = vec_perp[0:2]

            if np.isnan(avoid_dir):
                sub_target_1 = conv_position_2_list(pose_robot) + direction * len_along_path + vec_perp * self.res
                sub_target_2 = conv_position_2_list(pose_robot) + direction * len_along_path - vec_perp * self.res
                bool_sub_target_1 = self.verify_sub_target(sub_target_1)
                bool_sub_target_2 = self.verify_sub_target(sub_target_2)
                while bool_sub_target_1:
                    sub_target_1 += vec_perp * self.res
                    bool_sub_target_1 = self.verify_sub_target(sub_target_1)

                sub_target_1 += vec_perp * 0.01 * self.res
                while bool_sub_target_2:

                    sub_target_2 -= vec_perp * self.res
                    bool_sub_target_2 = self.verify_sub_target(sub_target_2)

                sub_target_2 -= vec_perp * 0.01 * self.res

                if abs(get_distance(path.start, sub_target_1) - get_distance(path.start, sub_target_2)) < 600:

                    sub_target = sub_target_1
                    avoid_dir = -vec_perp

                else:
                    if get_distance(path.start, sub_target_1) < get_distance(path.start, sub_target_2):
                        sub_target = sub_target_1
                        avoid_dir = -vec_perp

                    else:
                        sub_target = sub_target_2
                        avoid_dir = vec_perp

            else:
                if avoid_dir * np.transpose(vec_perp) < 0:
                    vec_perp = -vec_perp
                    sub_target = conv_position_2_list(pose_robot) + direction * len_along_path + vec_perp * self.res

                bool_sub_target = self.verify_sub_target(sub_target)
                while bool_sub_target:
                    sub_target -= vec_perp * self.res
                    bool_sub_target = self.verify_sub_target(sub_target)

                sub_target -= vec_perp * 0.01 * self.res
                avoid_dir = vec_perp

        else:
            sub_target = pose_target

        sub_target = Position(sub_target[0],sub_target[1])

        return [sub_target, avoid_dir]







































