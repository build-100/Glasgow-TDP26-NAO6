#!/usr/bin/env python3
import math
import random

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from soccer_field.soccer_2d_ball_physics import BallPhysics, SoccerGameRules
from soccer_field.soccer_2d_blue_team import BlueTeamBehavior
from soccer_field.soccer_2d_red_team import RedTeamBehavior


class Soccer2DSimulation(BlueTeamBehavior, RedTeamBehavior, BallPhysics):
    VISION_SEGMENTS = 36
    SEARCH_SPEED_FACTOR = 0.25
    UNSEEN_BALL_SPEED_FACTOR = 0.3
    TEAM_BALL_REPORT_TIMEOUT = 2.0
    MAX_ROBOT_SPEED = 2.0 / 3.6
    ROBOT_ACCELERATION = 0.3 / 3.6
    ROBOT_FALL_DURATION = 2.0
    ROBOT_COLLISION_CLEARANCE = 0.08
    ROBOT_COLLISION_STEP_BACK = 0.05
    STRIKER_PASS_DISTANCE = 0.5
    BOTH_STRIKERS_BALL_CONTACT_DURATION = 1.0
    MAX_BACKWARD_CARRY_DISTANCE = 1.0
    MIN_TEAMMATE_PASS_DISTANCE = 1.0
    PASS_RETURN_COOLDOWN = 4.0
    DEFENDER_MIDFIELD_BUFFER = 0.75
    OPPONENT_APPROACH_CLEARANCE = 0.1
    GOALIE_BALL_CONTACT_DURATION = 0.5
    GOALIE_PASS_SPEED_FACTOR = 0.5
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'soccer_field_link')
        self.field_length = rospy.get_param('~pitch_length', 9.0)
        self.field_width = rospy.get_param('~pitch_width', 6.0)
        self.robot_radius = rospy.get_param('~robot_radius', 0.25)
        self.robot_height = rospy.get_param('~robot_height', 0.65)
        self.ball_radius = rospy.get_param('~ball_radius', 0.11)
        self.robot_speed = rospy.get_param('~robot_speed', 1.0)
        self.obstacle_avoidance_distance = 0.20
        self.vision_range = rospy.get_param('~vision_range', 2.0)
        self.vision_aperture = math.radians(
            rospy.get_param('~vision_aperture_degrees', 120.0))
        self.goalie_vision_range = rospy.get_param(
            '~goalie_vision_range', 4.0)
        self.goalie_vision_aperture = math.radians(
            rospy.get_param('~goalie_vision_aperture_degrees', 150.0))
        self.approach_clearance = rospy.get_param(
            '~approach_clearance', 0.1)
        self.search_turn_rate = rospy.get_param('~search_turn_rate', 0.8)
        self.ball_speed_kmh = rospy.get_param('~ball_speed_kmh', 5.0)
        self.kick_speed_kmh = rospy.get_param('~kick_speed_kmh', 10.0)
        self.kick_duration = rospy.get_param('~kick_duration', 0.1)
        self.control_duration = rospy.get_param('~control_duration', 0.5)
        self.rolling_deceleration = rospy.get_param(
            '~rolling_deceleration', 0.15)
        self.wall_restitution = rospy.get_param('~wall_restitution', 0.8)
        self.goal_width = rospy.get_param('~goal_width', 2.6)
        self.goal_depth = rospy.get_param('~goal_depth', 0.6)
        self.goal_area_length = rospy.get_param('~goal_area_length', 1.0)
        self.goal_area_width = rospy.get_param('~goal_area_width', 3.0)
        self.goal_restart_delay = rospy.get_param('~goal_restart_delay', 0.9991)
        self.obstacles = rospy.get_param('~obstacles', [])
        random_seed = rospy.get_param('~random_seed', None)
        self.random_generator = random.Random(random_seed)
        self.opponent_goal_x = rospy.get_param(
            '~opponent_goal_x', self.field_length)

        if (self.field_length <= 2.0 * self.robot_radius or
                self.field_width <= 2.0 * self.robot_radius or
                self.robot_radius <= 0.0 or self.ball_radius <= 0.0 or
                self.robot_height <= 0.0 or self.robot_speed <= 0.0 or
                self.vision_range <= 0.0 or self.goalie_vision_range <= 0.0 or
                not 0.0 < self.vision_aperture <= 2.0 * math.pi or
                not 0.0 < self.goalie_vision_aperture <= 2.0 * math.pi or
                self.approach_clearance < 0.0 or self.search_turn_rate <= 0.0 or
                self.ball_speed_kmh < 0.0 or self.kick_speed_kmh <= 0.0 or
                self.kick_duration <= 0.0 or self.control_duration < 0.0 or
                self.goal_width <= 0.0 or self.goal_depth <= 0.0 or
                self.goal_area_length <= 2.0 * self.robot_radius or
                self.goal_area_length >= self.field_length or
                self.goal_area_width <= 2.0 * self.robot_radius or
                self.goal_area_width > self.field_width or
                self.goal_restart_delay < 0.0 or
                self.rolling_deceleration < 0.0 or
                not 0.0 <= self.wall_restitution <= 1.0 or
                not 0.0 <= self.opponent_goal_x <= self.field_length or
                not isinstance(self.obstacles, list)):
            raise rospy.ROSException(
                'Pitch, robot, ball, vision, and motion parameters must be '
                'positive and compatible')
        for obstacle in self.obstacles:
            if (not isinstance(obstacle, dict) or
                    not all(key in obstacle for key in ('x', 'y', 'radius')) or
                    not all(isinstance(obstacle[key], (int, float)) and
                            math.isfinite(obstacle[key])
                            for key in ('x', 'y', 'radius')) or
                    obstacle['radius'] <= 0.0):
                raise rospy.ROSException(
                    'Each obstacle must define numeric x, y, and positive radius')

        self.rules = SoccerGameRules(
            self.field_length, self.field_width,
            self.goal_width, self.ball_radius)
        self.robot_x = self.field_length * 0.25
        self.robot_y = 0.0
        self.robot_yaw = math.atan2(
            -self.robot_y, self.opponent_goal_x - self.robot_x)
        self.robot_current_speed = min(
            self.robot_speed * self.SEARCH_SPEED_FACTOR,
            self.MAX_ROBOT_SPEED)
        self.blue_goalie_x = self.blue_goalkeeper_home_x()
        self.blue_goalie_y = 0.0
        self.blue_goalie_yaw = 0.0
        self.blue_goalie_current_speed = self.robot_current_speed
        self.blue_goalie_fall_time = 0.0
        self.blue_goalie_ball_visible = False
        self.blue_goalie_ball_contact_elapsed = 0.0
        self.blue_defender_x = self.field_length * 0.15
        self.blue_defender_y = 0.0
        self.blue_defender_yaw = 0.0
        self.blue_defender_current_speed = self.robot_current_speed
        self.blue_defender_fall_time = 0.0
        self.red_defender_x = self.field_length * 0.85
        self.red_defender_y = 0.0
        self.red_defender_yaw = math.pi
        self.red_defender_current_speed = self.robot_current_speed
        self.red_defender_fall_time = 0.0
        self.constrain_defenders_to_own_half()
        self.defender_ball_holder = None
        self.defender_ball_hold_offset = None
        self.defender_carry_distance = 0.0
        self.red_striker_x = self.field_length * 0.75
        self.red_striker_y = 0.0
        self.red_striker_yaw = math.pi
        self.red_striker_current_speed = self.robot_current_speed
        self.red_striker_fall_time = 0.0
        self.red_striker_ball_visible = False
        self.opponent_x = self.goalkeeper_home_x()
        self.opponent_y = 0.0
        self.opponent_current_speed = self.robot_current_speed
        self.opponent_yaw = math.pi
        self.robot_fall_time = 0.0
        self.opponent_fall_time = 0.0
        self.opponent_ball_visible = False
        self.red_goalie_ball_contact_elapsed = 0.0
        self.search_waypoint_index = 0
        self.red_search_waypoint_index = 0
        self.initial_search_heading = self.robot_yaw
        self.red_initial_search_heading = math.atan2(
            -self.red_striker_y,
            self.blue_goal_line_x() - self.red_striker_x)
        self.initial_search_limit = math.pi * 0.5
        self.search_offset = 0.0
        self.red_search_offset = 0.0
        self.search_direction = 1.0
        self.red_search_direction = -1.0
        self.team_ball_reports = {'Blue': None, 'Red': None}
        self.team_robot_reports = {'Blue': {}, 'Red': {}}
        self.last_pass = None
        self.simulation_time = 0.0
        self.blue_sees_red = False
        self.red_sees_blue = False
        self.has_detected_ball = False
        self.red_has_detected_ball = False
        self.ball_x = self.field_length * 0.5
        self.ball_y = 0.0
        ball_angle = self.random_generator.uniform(-math.pi, math.pi)
        ball_speed = self.ball_speed_kmh / 3.6
        self.ball_vx = ball_speed * math.cos(ball_angle)
        self.ball_vy = ball_speed * math.sin(ball_angle)
        self.ball_goal_side = None
        self.goal_restart_elapsed = 0.0
        self.left_score = 0
        self.right_score = 0
        self.ball_detected = False
        self.kick_active = False
        self.kick_elapsed = 0.0
        self.kick_direction = (1.0, 0.0)
        self.ball_controlled = False
        self.ball_controlled_team = None
        self.striker_carry_distance = 0.0
        self.control_elapsed = 0.0
        self.goalie_ball_holder = None
        self.goalie_ball_hold_offset = None
        self.both_strikers_ball_contact_elapsed = 0.0
        self.kick_target_y = 0.0
        self.kick_target = None
        self.publisher = rospy.Publisher(
            '/soccer_field/simulation', MarkerArray, queue_size=1)
        self.last_update = None
        self.timer = rospy.Timer(rospy.Duration(1.0 / 30.0), self.update)
        self.publish_markers()
        rospy.loginfo(
            '2D simulation ready: ball launched at %.1f km/h; striker vision '
            'is %.1f m / %.0f degrees; goalie vision is %.1f m / %.0f degrees',
            self.ball_speed_kmh, self.vision_range,
            math.degrees(self.vision_aperture), self.goalie_vision_range,
            math.degrees(self.goalie_vision_aperture))

    def ball_in_vision(self):
        return self.object_in_vision(
            self.robot_x, self.robot_y, self.robot_yaw)

    def object_in_vision(self, x, y, yaw):
        return self.point_in_vision(
            self.ball_x, self.ball_y, x, y, yaw)

    def point_in_vision(self, target_x, target_y, x, y, yaw,
                        vision_range=None, vision_aperture=None):
        if vision_range is None:
            vision_range = self.vision_range
        if vision_aperture is None:
            vision_aperture = self.vision_aperture
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        if distance > vision_range:
            return False
        if distance == 0.0:
            return True
        bearing = math.atan2(dy, dx)
        angle = math.atan2(
            math.sin(bearing - yaw),
            math.cos(bearing - yaw))
        return abs(angle) <= vision_aperture * 0.5

    def goalkeeper_home_x(self):
        return min(
            max(self.field_length - self.goal_area_length * 0.35,
                self.field_length - self.goal_area_length +
                self.robot_radius),
            self.field_length - self.robot_radius)

    def blue_goalkeeper_home_x(self):
        return max(
            min(self.goal_area_length * 0.35,
                self.goal_area_length - self.robot_radius),
            self.robot_radius)

    def blue_goal_line_x(self):
        return (0.0 if self.opponent_goal_x > self.field_length * 0.5
                else self.field_length)

    def constrain_goalkeeper_to_area(self):
        self.opponent_x = min(
            max(self.opponent_x,
                self.field_length - self.goal_area_length +
                self.robot_radius),
            self.field_length - self.robot_radius)
        half_width = self.goal_area_width * 0.5 - self.robot_radius
        self.opponent_y = min(max(self.opponent_y, -half_width), half_width)

    def constrain_blue_goalkeeper_to_area(self):
        self.blue_goalie_x = min(
            max(self.blue_goalie_x, self.robot_radius),
            self.goal_area_length - self.robot_radius)
        half_width = self.goal_area_width * 0.5 - self.robot_radius
        self.blue_goalie_y = min(
            max(self.blue_goalie_y, -half_width), half_width)

    def constrain_defenders_to_own_half(self):
        halfway = self.field_length * 0.5
        self.blue_defender_x = min(
            max(self.blue_defender_x, self.robot_radius),
            halfway - self.robot_radius)
        self.blue_defender_y = min(
            max(self.blue_defender_y,
                -self.field_width * 0.5 + self.robot_radius),
            self.field_width * 0.5 - self.robot_radius)
        self.red_defender_x = min(
            max(self.red_defender_x, halfway + self.robot_radius),
            self.field_length - self.robot_radius)
        self.red_defender_y = min(
            max(self.red_defender_y,
                -self.field_width * 0.5 + self.robot_radius),
            self.field_width * 0.5 - self.robot_radius)

    def update_robot_detection(self):
        blue_sees_red = self.point_in_vision(
            self.opponent_x, self.opponent_y,
            self.robot_x, self.robot_y, self.robot_yaw)
        red_sees_blue = self.point_in_vision(
            self.robot_x, self.robot_y,
            self.opponent_x, self.opponent_y, self.opponent_yaw,
            self.goalie_vision_range, self.goalie_vision_aperture)

        if blue_sees_red and not self.blue_sees_red:
            rospy.loginfo(
                'Blue Striker detected Red Goalie: range %.2f m, bearing %.1f deg',
                math.hypot(
                    self.opponent_x - self.robot_x,
                    self.opponent_y - self.robot_y),
                math.degrees(math.atan2(
                    self.opponent_y - self.robot_y,
                    self.opponent_x - self.robot_x)))
        if red_sees_blue and not self.red_sees_blue:
            rospy.loginfo(
                'Red Goalie detected Blue Striker: range %.2f m, bearing %.1f deg',
                math.hypot(
                    self.robot_x - self.opponent_x,
                    self.robot_y - self.opponent_y),
                math.degrees(math.atan2(
                    self.robot_y - self.opponent_y,
                    self.robot_x - self.opponent_x)))
        self.blue_sees_red = blue_sees_red
        self.red_sees_blue = red_sees_blue

    def update_team_ball_reports(self):
        robots = (
            ('Blue', 'Striker', self.robot_x, self.robot_y,
             self.robot_yaw, self.vision_range, self.vision_aperture),
            ('Blue', 'Goalie', self.blue_goalie_x, self.blue_goalie_y,
             self.blue_goalie_yaw, self.goalie_vision_range,
             self.goalie_vision_aperture),
            ('Blue', 'Defender', self.blue_defender_x, self.blue_defender_y,
             self.blue_defender_yaw, self.vision_range, self.vision_aperture),
            ('Red', 'Striker', self.red_striker_x, self.red_striker_y,
             self.red_striker_yaw, self.vision_range, self.vision_aperture),
            ('Red', 'Goalie', self.opponent_x, self.opponent_y,
             self.opponent_yaw, self.goalie_vision_range,
             self.goalie_vision_aperture),
            ('Red', 'Defender', self.red_defender_x, self.red_defender_y,
             self.red_defender_yaw, self.vision_range, self.vision_aperture),
        )
        ball_seen_by_team = {'Blue': False, 'Red': False}
        for team, role, x, y, yaw, vision_range, vision_aperture in robots:
            self.team_robot_reports[team][role] = (x, y, yaw)
            if self.point_in_vision(
                    self.ball_x, self.ball_y, x, y, yaw,
                    vision_range, vision_aperture):
                ball_seen_by_team[team] = True
        for team, sees_ball in ball_seen_by_team.items():
            if sees_ball:
                self.team_ball_reports[team] = (
                    self.ball_x, self.ball_y, self.ball_vx, self.ball_vy,
                    self.simulation_time)

    def reflected_coordinate(self, position, velocity, elapsed, minimum,
                             maximum):
        span = maximum - minimum
        if span <= 0.0:
            return minimum
        phase = (position - minimum + velocity * elapsed) % (2.0 * span)
        if phase > span:
            return maximum - (phase - span)
        return minimum + phase

    def shared_ball_position(self, team):
        report = self.team_ball_reports[team]
        if report is None:
            return None, None
        x, y, velocity_x, velocity_y, observed_at = report
        elapsed = max(0.0, self.simulation_time - observed_at)
        if elapsed > self.TEAM_BALL_REPORT_TIMEOUT:
            return None, None
        speed = math.hypot(velocity_x, velocity_y)
        if speed > 0.0 and self.rolling_deceleration > 0.0:
            prediction_time = min(
                elapsed, speed / self.rolling_deceleration)
            prediction_time -= (
                self.rolling_deceleration * prediction_time ** 2 /
                (2.0 * speed))
        else:
            prediction_time = elapsed
        return (
            self.reflected_coordinate(
                x, velocity_x, prediction_time, self.ball_radius,
                self.field_length - self.ball_radius),
            self.reflected_coordinate(
                y, velocity_y, prediction_time,
                -self.field_width * 0.5 + self.ball_radius,
                self.field_width * 0.5 - self.ball_radius))

    def known_ball_position(self, team, x, y, yaw, is_goalie=False):
        vision_range = (self.goalie_vision_range if is_goalie
                        else self.vision_range)
        vision_aperture = (self.goalie_vision_aperture if is_goalie
                           else self.vision_aperture)
        if self.point_in_vision(
                self.ball_x, self.ball_y, x, y, yaw,
                vision_range, vision_aperture):
            return self.ball_x, self.ball_y, True
        ball_x, ball_y = self.shared_ball_position(team)
        if ball_x is None:
            return None, None, False
        return ball_x, ball_y, False

    def teammate_positions(self, team, role):
        return {
            other_role: position
            for other_role, position in self.team_robot_reports[team].items()
            if other_role != role
        }

    def closest_pursuer(self, team, ball_x, ball_y):
        if team == 'Blue':
            striker_x, striker_y = self.robot_x, self.robot_y
            defender_x, defender_y = (
                self.blue_defender_x, self.blue_defender_y)
        else:
            striker_x, striker_y = self.red_striker_x, self.red_striker_y
            defender_x, defender_y = (
                self.red_defender_x, self.red_defender_y)
        striker_distance = math.hypot(
            ball_x - striker_x, ball_y - striker_y)
        defender_distance = math.hypot(
            ball_x - defender_x, ball_y - defender_y)
        return 'Defender' if defender_distance < striker_distance else 'Striker'

    def opponent_ball_holder_position(self, team):
        opposing_team = 'Red' if team == 'Blue' else 'Blue'
        if (self.ball_controlled and
                self.ball_controlled_team == opposing_team):
            return (
                (self.red_striker_x, self.red_striker_y)
                if opposing_team == 'Red' else
                (self.robot_x, self.robot_y))
        if self.goalie_ball_holder == opposing_team:
            return (
                (self.opponent_x, self.opponent_y)
                if opposing_team == 'Red' else
                (self.blue_goalie_x, self.blue_goalie_y))
        if self.defender_ball_holder == opposing_team:
            return (
                (self.red_defender_x, self.red_defender_y)
                if opposing_team == 'Red' else
                (self.blue_defender_x, self.blue_defender_y))
        return None

    def team_sees_opponent_ball_holder(self, team, ball_holder):
        for role, (x, y, yaw) in self.team_robot_reports[team].items():
            is_goalie = role == 'Goalie'
            vision_range = (
                self.goalie_vision_range if is_goalie else self.vision_range)
            vision_aperture = (
                self.goalie_vision_aperture
                if is_goalie else self.vision_aperture)
            if self.point_in_vision(
                    ball_holder[0], ball_holder[1], x, y, yaw,
                    vision_range, vision_aperture):
                return True
        return False

    def collision_avoiding_step(self, x, y, delta_x, delta_y):
        distance = math.hypot(delta_x, delta_y)
        if distance == 0.0:
            return 0.0, 0.0

        direction_x = delta_x / distance
        direction_y = delta_y / distance
        robots = [
            (self.robot_x, self.robot_y),
            (self.opponent_x, self.opponent_y),
            (self.blue_goalie_x, self.blue_goalie_y),
            (self.red_striker_x, self.red_striker_y),
            (self.blue_defender_x, self.blue_defender_y),
            (self.red_defender_x, self.red_defender_y),
        ]
        blocking_robots = []
        for other_x, other_y in robots:
            if math.hypot(other_x - x, other_y - y) <= 1e-9:
                continue
            relative_x = other_x - x
            relative_y = other_y - y
            along = relative_x * direction_x + relative_y * direction_y
            distance_squared = relative_x * relative_x + relative_y * relative_y
            radius = (
                2.0 * self.robot_radius + self.ROBOT_COLLISION_CLEARANCE)
            if distance_squared < radius * radius:
                if along >= 0.0:
                    blocking_robots.append(
                        (relative_x, relative_y, radius, distance_squared))
                continue
            perpendicular_squared = (
                relative_x * relative_x + relative_y * relative_y -
                along * along)
            discriminant = radius * radius - perpendicular_squared
            if discriminant >= 0.0:
                entry_distance = along - math.sqrt(discriminant)
                if 0.0 <= entry_distance <= distance:
                    blocking_robots.append(
                        (relative_x, relative_y, radius, distance_squared))

        if not blocking_robots:
            return direction_x * distance, direction_y * distance

        candidate_directions = []
        for relative_x, relative_y, radius, distance_squared in blocking_robots:
            center_distance = math.sqrt(distance_squared)
            obstacle_bearing = math.atan2(relative_y, relative_x)
            tangent_offset = (
                math.pi * 0.5 if center_distance <= radius + 0.02 else
                math.asin(radius / center_distance))
            candidate_directions.extend((
                (math.cos(obstacle_bearing + tangent_offset),
                 math.sin(obstacle_bearing + tangent_offset)),
                (math.cos(obstacle_bearing - tangent_offset),
                 math.sin(obstacle_bearing - tangent_offset)),
            ))

        best_step = (0.0, 0.0)
        best_progress = -math.inf
        for candidate_x, candidate_y in candidate_directions:
            max_distance = distance
            for other_x, other_y in robots:
                relative_x = other_x - x
                relative_y = other_y - y
                if math.hypot(relative_x, relative_y) <= 1e-9:
                    continue
                along = relative_x * candidate_x + relative_y * candidate_y
                if along <= 1e-6:
                    continue
                perpendicular_squared = max(
                    0.0,
                    relative_x * relative_x + relative_y * relative_y -
                    along * along)
                radius = (
                    2.0 * self.robot_radius + self.ROBOT_COLLISION_CLEARANCE)
                discriminant = radius * radius - perpendicular_squared
                if discriminant < 0.0:
                    continue
                entry_distance = along - math.sqrt(discriminant)
                if 0.0 <= entry_distance <= max_distance:
                    max_distance = min(
                        max_distance, max(0.0, entry_distance - 0.02))

            if max_distance <= 0.0:
                continue
            step_x = candidate_x * max_distance
            step_y = candidate_y * max_distance
            progress = distance - math.hypot(
                delta_x - step_x, delta_y - step_y)
            if progress > best_progress:
                best_progress = progress
                best_step = (step_x, step_y)
        return best_step

    def ball_target(self, ball_x=None, ball_y=None, goal_x=None):
        if ball_x is None:
            ball_x = self.ball_x
        if ball_y is None:
            ball_y = self.ball_y
        if goal_x is None:
            goal_x = self.opponent_goal_x
        clearance = self.robot_radius + self.ball_radius
        goal_dx = goal_x - ball_x
        goal_dy = -ball_y
        goal_distance = math.hypot(goal_dx, goal_dy)
        if goal_distance == 0.0:
            return ball_x - clearance, ball_y
        return (
            ball_x - goal_dx * clearance / goal_distance,
            ball_y - goal_dy * clearance / goal_distance)

    def select_best_shot(self, goal_x=None):
        if goal_x is None:
            goal_x = self.opponent_goal_x
        goal_min_y = -self.goal_width * 0.5 + self.ball_radius
        goal_max_y = self.goal_width * 0.5 - self.ball_radius
        best_target = None
        best_clearance = -math.inf
        for index in range(21):
            end_y = goal_min_y + (goal_max_y - goal_min_y) * index / 20.0
            shot_clearance = self.shot_clearance_to_goal(end_y, goal_x)
            if (shot_clearance is not None and
                    (shot_clearance > best_clearance or
                     (math.isclose(shot_clearance, best_clearance) and
                      abs(end_y) < abs(best_target[1])))):
                best_clearance = shot_clearance
                best_target = (goal_x, end_y)
        return best_target

    def shot_clearance_to_goal(self, target_y, goal_x=None):
        if goal_x is None:
            goal_x = self.opponent_goal_x
        goal_min_y = -self.goal_width * 0.5 + self.ball_radius
        goal_max_y = self.goal_width * 0.5 - self.ball_radius
        dx = goal_x - self.ball_x
        if (goal_min_y > goal_max_y or dx == 0.0 or
                not goal_min_y <= target_y <= goal_max_y):
            return None

        dy = target_y - self.ball_y
        angle = math.atan2(dy * math.copysign(1.0, dx), abs(dx))
        if abs(angle) > math.radians(70.0):
            return None

        length_squared = dx * dx + dy * dy
        clearance = math.inf
        if goal_x > self.ball_x:
            blockers = (
                (self.red_striker_x, self.red_striker_y),
                (self.opponent_x, self.opponent_y),
                (self.red_defender_x, self.red_defender_y),
            )
        else:
            blockers = (
                (self.robot_x, self.robot_y),
                (self.blue_goalie_x, self.blue_goalie_y),
                (self.blue_defender_x, self.blue_defender_y),
            )
        for blocker_x, blocker_y in blockers:
            projection = (
                (blocker_x - self.ball_x) * dx +
                (blocker_y - self.ball_y) * dy) / length_squared
            projection = min(max(projection, 0.0), 1.0)
            closest_x = self.ball_x + projection * dx
            closest_y = self.ball_y + projection * dy
            blocker_clearance = (
                math.hypot(blocker_x - closest_x, blocker_y - closest_y) -
                self.robot_radius - self.ball_radius)
            clearance = min(clearance, blocker_clearance)
        for obstacle in self.obstacles:
            projection = (
                (obstacle['x'] - self.ball_x) * dx +
                (obstacle['y'] - self.ball_y) * dy) / length_squared
            projection = min(max(projection, 0.0), 1.0)
            closest_x = self.ball_x + projection * dx
            closest_y = self.ball_y + projection * dy
            obstacle_clearance = (
                math.hypot(obstacle['x'] - closest_x,
                           obstacle['y'] - closest_y) -
                obstacle['radius'] - self.ball_radius)
            clearance = min(clearance, obstacle_clearance)
        return clearance if clearance >= 0.0 else None

    def shot_is_clear(self):
        return self.select_best_shot() is not None

    def robot_touches_ball(self):
        contact_distance = self.robot_radius + self.ball_radius
        robot_ball_distance = math.hypot(
            self.ball_x - self.robot_x, self.ball_y - self.robot_y)
        return robot_ball_distance <= contact_distance + 0.02

    def wall_escape_direction(self):
        min_x = self.ball_radius
        max_x = self.field_length - self.ball_radius
        min_y = -self.field_width * 0.5 + self.ball_radius
        max_y = self.field_width * 0.5 - self.ball_radius
        walls = (
            (self.ball_x - min_x, 1.0, 0.0),
            (max_x - self.ball_x, -1.0, 0.0),
            (self.ball_y - min_y, 0.0, 1.0),
            (max_y - self.ball_y, 0.0, -1.0),
        )
        wall_distance, normal_x, normal_y = min(
            walls, key=lambda wall: wall[0])
        if wall_distance > 0.15:
            return None
        return normal_x, normal_y

    def start_wall_escape_kick(self, team='Blue'):
        direction = self.wall_escape_direction()
        if direction is None:
            return False

        direction_x, direction_y = direction
        separation = self.robot_radius + self.ball_radius
        is_blue = team == 'Blue'
        x_attribute = 'robot_x' if is_blue else 'red_striker_x'
        y_attribute = 'robot_y' if is_blue else 'red_striker_y'
        yaw_attribute = 'robot_yaw' if is_blue else 'red_striker_yaw'
        self.kick_direction = direction
        self.kick_elapsed = 0.0
        self.kick_active = True
        self.ball_controlled = False
        self.ball_controlled_team = None
        self.striker_carry_distance = 0.0
        setattr(self, x_attribute, self.ball_x - direction_x * separation)
        setattr(self, y_attribute, self.ball_y - direction_y * separation)
        setattr(
            self, yaw_attribute, math.atan2(direction_y, direction_x))
        self.ball_vx = 0.0
        self.ball_vy = 0.0
        rospy.loginfo(
            '%s Striker kicks trapped ball away from the wall', team)
        return True

    def is_facing_opponent_goal(
            self, target_y, actor='Blue', goal_x=None):
        if goal_x is None:
            goal_x = (
                self.opponent_goal_x if actor == 'Blue'
                else self.blue_goal_line_x())
        robot_x, robot_y, robot_yaw = (
            (self.robot_x, self.robot_y, self.robot_yaw)
            if actor == 'Blue' else
            (self.red_striker_x, self.red_striker_y, self.red_striker_yaw))
        goal_bearing = math.atan2(
            target_y - robot_y, goal_x - robot_x)
        heading_error = self.normalize_angle(goal_bearing - robot_yaw)
        return abs(heading_error) <= math.radians(20.0)

    def can_kick_to_goal(self, target_y, actor='Blue', goal_x=None):
        if goal_x is None:
            goal_x = (
                self.opponent_goal_x if actor == 'Blue'
                else self.blue_goal_line_x())
        robot_x, robot_y = (
            (self.robot_x, self.robot_y) if actor == 'Blue' else
            (self.red_striker_x, self.red_striker_y))
        touches_ball = (
            self.robot_touches_ball() if actor == 'Blue'
            else self.red_striker_touches_ball())
        if (not touches_ball or
                self.shot_clearance_to_goal(target_y, goal_x) is None or
                not self.is_facing_opponent_goal(
                    target_y, actor, goal_x)):
            return False

        robot_to_goal_x = goal_x - robot_x
        robot_to_goal_y = target_y - robot_y
        distance_squared = (
            robot_to_goal_x * robot_to_goal_x +
            robot_to_goal_y * robot_to_goal_y)
        if distance_squared == 0.0:
            return False

        ball_from_robot_x = self.ball_x - robot_x
        ball_from_robot_y = self.ball_y - robot_y
        projection = (
            ball_from_robot_x * robot_to_goal_x +
            ball_from_robot_y * robot_to_goal_y) / distance_squared
        cross_product = (
            robot_to_goal_x * ball_from_robot_y -
            robot_to_goal_y * ball_from_robot_x)
        distance_from_goal_line = abs(cross_product) / math.sqrt(
            distance_squared)
        return (
            0.0 < projection < 1.0 and
            distance_from_goal_line <=
            self.robot_radius + self.ball_radius + 0.02)

    def start_kick(self, actor='Blue', goal_x=None):
        if goal_x is None:
            goal_x = (
                self.opponent_goal_x if actor == 'Blue'
                else self.blue_goal_line_x())
        if not self.can_kick_to_goal(
                self.kick_target_y, actor, goal_x):
            rospy.logwarn(
                'Kick requires ball contact and a clear, aligned shot into '
                'the goal')
            self.ball_controlled = False
            self.ball_controlled_team = None
            self.striker_carry_distance = 0.0
            return

        goal_dx = goal_x - self.ball_x
        goal_dy = self.kick_target_y - self.ball_y
        goal_distance = math.hypot(goal_dx, goal_dy)
        self.kick_direction = (
            goal_dx / goal_distance, goal_dy / goal_distance)
        self.kick_elapsed = 0.0
        self.kick_active = True
        self.ball_controlled = False
        self.ball_controlled_team = None
        self.striker_carry_distance = 0.0
        self.ball_vx = 0.0
        self.ball_vy = 0.0
        self.set_robot_in_contact(actor)
        rospy.loginfo(
            'Starting %.1f second %s striker kick toward the goal',
            self.kick_duration, actor)

    def control_ball(self, target, actor='Blue', goal_x=None):
        if goal_x is None:
            goal_x = (
                self.opponent_goal_x if actor == 'Blue'
                else self.blue_goal_line_x())
        self.ball_controlled = True
        self.ball_controlled_team = actor
        self.striker_carry_distance = 0.0
        self.control_elapsed = 0.0
        self.kick_target = target
        self.kick_target_y = target[1]
        self.ball_vx = 0.0
        self.ball_vy = 0.0
        direction = (goal_x - self.ball_x,
                     self.kick_target_y - self.ball_y)
        distance = math.hypot(*direction)
        if distance > 0.0:
            dx, dy = direction[0] / distance, direction[1] / distance
            separation = self.robot_radius + self.ball_radius
            if actor == 'Blue':
                candidate_x = self.ball_x - dx * separation
                candidate_y = self.ball_y - dy * separation
                other_robots = (
                    (self.opponent_x, self.opponent_y),
                    (self.blue_goalie_x, self.blue_goalie_y),
                    (self.red_striker_x, self.red_striker_y),
                    (self.blue_defender_x, self.blue_defender_y),
                    (self.red_defender_x, self.red_defender_y),
                )
                if all(
                        math.hypot(candidate_x - x, candidate_y - y) >=
                        2.0 * self.robot_radius +
                        self.ROBOT_COLLISION_CLEARANCE
                        for x, y in other_robots):
                    self.robot_x = candidate_x
                    self.robot_y = candidate_y
                self.robot_yaw = math.atan2(dy, dx)
            else:
                candidate_x = self.ball_x - dx * separation
                candidate_y = self.ball_y - dy * separation
                other_robots = (
                    (self.robot_x, self.robot_y),
                    (self.opponent_x, self.opponent_y),
                    (self.blue_goalie_x, self.blue_goalie_y),
                    (self.blue_defender_x, self.blue_defender_y),
                    (self.red_defender_x, self.red_defender_y),
                )
                if all(
                        math.hypot(candidate_x - x, candidate_y - y) >=
                        2.0 * self.robot_radius +
                        self.ROBOT_COLLISION_CLEARANCE
                        for x, y in other_robots):
                    self.red_striker_x = candidate_x
                    self.red_striker_y = candidate_y
                self.red_striker_yaw = math.atan2(dy, dx)
        rospy.loginfo(
            '%s Striker controls the ball; carry limit is %.1f m',
            actor, self.MAX_BACKWARD_CARRY_DISTANCE)

    def update_control(self, dt):
        team = self.ball_controlled_team
        if team not in ('Blue', 'Red'):
            raise rospy.ROSException(
                'Controlled ball is missing its striker team')
        goal_x = (
            self.opponent_goal_x if team == 'Blue'
            else self.blue_goal_line_x())
        target = self.select_best_shot(goal_x)
        if target is not None:
            self.kick_target_y = target[1]
            if self.can_kick_to_goal(target[1], team, goal_x):
                self.start_kick(team, goal_x)
                return
            moved = self.move_striker_into_shot_position(team, target, dt)
            if not moved:
                self.pass_ball_to_teammate(
                    team, passer='Striker', receiver='Defender')
            return
        moved = self.move_ball_carrier_to_pass_lane(
            team, 'Striker', 'Defender', dt, 'striker_carry_distance')
        if (not moved or (
                self.ball_clearance_from_robots(
                    team, 'Striker', self.ball_x, self.ball_y) <= 0.05 and
                self.striker_carry_distance >=
                self.MAX_BACKWARD_CARRY_DISTANCE)):
            self.pass_ball_to_teammate(
                team, passer='Striker', receiver='Defender')

    def move_striker_into_shot_position(self, team, target, dt):
        if team == 'Blue':
            x_attribute, y_attribute, yaw_attribute = (
                'robot_x', 'robot_y', 'robot_yaw')
        else:
            x_attribute, y_attribute, yaw_attribute = (
                'red_striker_x', 'red_striker_y', 'red_striker_yaw')
        x = getattr(self, x_attribute)
        y = getattr(self, y_attribute)
        old_x, old_y = x, y
        direction_x = target[0] - self.ball_x
        direction_y = target[1] - self.ball_y
        length = math.hypot(direction_x, direction_y)
        if length <= 1e-9:
            return
        direction_x /= length
        direction_y /= length
        setattr(
            self, yaw_attribute, math.atan2(direction_y, direction_x))
        separation = self.robot_radius + self.ball_radius
        target_x = self.ball_x - direction_x * separation
        target_y = self.ball_y - direction_y * separation
        dx, dy = target_x - x, target_y - y
        distance = math.hypot(dx, dy)
        if distance > 0.01:
            step = min(self.MAX_ROBOT_SPEED * dt, distance)
            dx, dy = self.collision_avoiding_step(
                x, y, dx * step / distance, dy * step / distance)
            setattr(
                self, x_attribute,
                min(max(x + dx, self.robot_radius),
                    self.field_length - self.robot_radius))
            setattr(
                self, y_attribute,
                min(max(y + dy, -self.field_width * 0.5 + self.robot_radius),
                    self.field_width * 0.5 - self.robot_radius))
        x = getattr(self, x_attribute)
        y = getattr(self, y_attribute)
        self.ball_x = x + direction_x * separation
        self.ball_y = y + direction_y * separation
        self.ball_vx = 0.0
        self.ball_vy = 0.0
        return math.hypot(
            getattr(self, x_attribute) - old_x,
            getattr(self, y_attribute) - old_y) > 1e-9

    def set_robot_in_contact(self, actor='Blue'):
        dx, dy = self.kick_direction
        separation = self.robot_radius + self.ball_radius
        if actor == 'Blue':
            self.robot_x = self.ball_x - dx * separation
            self.robot_y = self.ball_y - dy * separation
            self.robot_yaw = math.atan2(dy, dx)
        else:
            self.red_striker_x = self.ball_x - dx * separation
            self.red_striker_y = self.ball_y - dy * separation
            self.red_striker_yaw = math.atan2(dy, dx)

    def update_kick(self, dt):
        self.kick_elapsed = min(self.kick_elapsed + dt, self.kick_duration)
        if self.kick_elapsed >= self.kick_duration - 1e-9:
            self.kick_elapsed = self.kick_duration
            self.kick_active = False
            kick_speed = self.kick_speed_kmh / 3.6
            self.ball_vx = self.kick_direction[0] * kick_speed
            self.ball_vy = self.kick_direction[1] * kick_speed
            rospy.loginfo('Kick complete; ball speed is %.1f km/h',
                          self.kick_speed_kmh)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def update_striker(self, team, dt):
        is_blue = team == 'Blue'
        x_attribute = 'robot_x' if is_blue else 'red_striker_x'
        y_attribute = 'robot_y' if is_blue else 'red_striker_y'
        yaw_attribute = 'robot_yaw' if is_blue else 'red_striker_yaw'
        speed_attribute = (
            'robot_current_speed' if is_blue else 'red_striker_current_speed')
        carrier = None
        if self.defender_ball_holder == team:
            carrier = (
                (self.blue_defender_x, self.blue_defender_y)
                if is_blue else
                (self.red_defender_x, self.red_defender_y))
        elif self.goalie_ball_holder == team:
            carrier = (
                (self.blue_goalie_x, self.blue_goalie_y)
                if is_blue else
                (self.opponent_x, self.opponent_y))
        if carrier is not None:
            self.move_striker_to_support_position(team, carrier, dt)
            return
        visible_attribute = (
            'ball_detected' if is_blue else 'red_striker_ball_visible')
        x = getattr(self, x_attribute)
        y = getattr(self, y_attribute)
        yaw = getattr(self, yaw_attribute)
        goal_x = self.opponent_goal_x if is_blue else self.blue_goal_line_x()
        target_ball_x, target_ball_y, sees_ball = self.known_ball_position(
            team, x, y, yaw)
        if target_ball_x is None:
            setattr(self, visible_attribute, False)
            self.search_for_team_ball(team, dt)
            return

        previously_visible = getattr(self, visible_attribute)
        if sees_ball:
            detected_attribute = (
                'has_detected_ball' if is_blue
                else 'red_has_detected_ball')
            setattr(self, detected_attribute, True)
        if sees_ball and not previously_visible:
            rospy.loginfo(
                '%s Striker sees ball: range %.2f m, bearing %.1f deg',
                team, math.hypot(target_ball_x - x, target_ball_y - y),
                math.degrees(math.atan2(target_ball_y - y, target_ball_x - x)))
        setattr(self, visible_attribute, sees_ball)
        if self.closest_pursuer(team, target_ball_x, target_ball_y) != 'Striker':
            return

        speed = getattr(self, speed_attribute)
        if sees_ball:
            speed = min(
                speed + self.ROBOT_ACCELERATION * dt,
                self.MAX_ROBOT_SPEED)
        else:
            speed = max(
                min(
                    self.robot_speed * self.UNSEEN_BALL_SPEED_FACTOR,
                    self.MAX_ROBOT_SPEED),
                speed - self.ROBOT_ACCELERATION * dt)
        setattr(self, speed_attribute, speed)

        target_x, target_y = self.ball_target(
            target_ball_x, target_ball_y, goal_x)
        opponent_striker_x, opponent_striker_y = (
            (self.red_striker_x, self.red_striker_y) if is_blue else
            (self.robot_x, self.robot_y))
        target_from_opponent_x = target_x - opponent_striker_x
        target_from_opponent_y = target_y - opponent_striker_y
        target_from_opponent = math.hypot(
            target_from_opponent_x, target_from_opponent_y)
        minimum_opponent_clearance = (
            2.0 * self.robot_radius + self.OPPONENT_APPROACH_CLEARANCE)
        if target_from_opponent < minimum_opponent_clearance:
            if target_from_opponent <= 1e-9:
                target_from_opponent_x = x - opponent_striker_x
                target_from_opponent_y = y - opponent_striker_y
                target_from_opponent = math.hypot(
                    target_from_opponent_x, target_from_opponent_y)
            if target_from_opponent <= 1e-9:
                target_from_opponent_x, target_from_opponent_y = 0.0, 1.0
                target_from_opponent = 1.0
            target_x = (
                opponent_striker_x + target_from_opponent_x *
                minimum_opponent_clearance / target_from_opponent)
            target_y = (
                opponent_striker_y + target_from_opponent_y *
                minimum_opponent_clearance / target_from_opponent)
            target_x = min(
                max(target_x, self.robot_radius),
                self.field_length - self.robot_radius)
            target_y = min(
                max(target_y, -self.field_width * 0.5 + self.robot_radius),
                self.field_width * 0.5 - self.robot_radius)
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        if distance > 0.01:
            setattr(
                self, yaw_attribute,
                math.atan2(target_ball_y - y, target_ball_x - x))
            step = min(speed * dt, distance)
            if is_blue:
                velocity_x, velocity_y = self.move_robot(
                    False, dx * step / distance,
                    dy * step / distance, dt)
                self.resolve_robot_ball_collision(velocity_x, velocity_y)
            else:
                velocity_x, velocity_y = self.move_extra_robot(
                    'red_striker', dx * step / distance,
                    dy * step / distance, dt)
                self.resolve_actor_ball_collision(
                    'red_striker', velocity_x, velocity_y)

        x = getattr(self, x_attribute)
        y = getattr(self, y_attribute)
        if distance > 0.01:
            yaw = math.atan2(target_ball_y - y, target_ball_x - x)
        else:
            yaw = math.atan2(-y, goal_x - x)
        setattr(self, yaw_attribute, yaw)

        touches_ball = (
            self.robot_touches_ball() if is_blue
            else self.red_striker_touches_ball())
        if not (sees_ball and distance <= 0.01 and touches_ball):
            return
        if self.start_wall_escape_kick(team):
            return
        target = self.select_best_shot(goal_x)
        if target is None:
            target = (goal_x, self.ball_y)
        self.control_ball(target, team, goal_x)

    def move_striker_to_support_position(self, team, carrier, dt):
        is_blue = team == 'Blue'
        x_attribute = 'robot_x' if is_blue else 'red_striker_x'
        y_attribute = 'robot_y' if is_blue else 'red_striker_y'
        yaw_attribute = 'robot_yaw' if is_blue else 'red_striker_yaw'
        speed_attribute = (
            'robot_current_speed' if is_blue else 'red_striker_current_speed')
        attack_direction = 1.0 if is_blue else -1.0
        support_distance = min(1.2, self.field_length * 0.2)
        target_x = carrier[0] + attack_direction * support_distance
        target_y = carrier[1]
        target_x = min(
            max(target_x, self.robot_radius),
            self.field_length - self.robot_radius)
        target_y = min(
            max(target_y, -self.field_width * 0.5 + self.robot_radius),
            self.field_width * 0.5 - self.robot_radius)

        x = getattr(self, x_attribute)
        y = getattr(self, y_attribute)
        dx, dy = target_x - x, target_y - y
        distance = math.hypot(dx, dy)
        speed = min(
            getattr(self, speed_attribute) + self.ROBOT_ACCELERATION * dt,
            self.MAX_ROBOT_SPEED)
        setattr(self, speed_attribute, speed)
        if distance > 0.01:
            step = min(speed * dt, distance)
            delta_x, delta_y = self.collision_avoiding_step(
                x, y, dx * step / distance, dy * step / distance)
            if is_blue:
                self.move_robot(False, delta_x, delta_y, dt)
            else:
                self.move_extra_robot(
                    'red_striker', delta_x, delta_y, dt)
        setattr(
            self, yaw_attribute,
            math.atan2(carrier[1] - getattr(self, y_attribute),
                       carrier[0] - getattr(self, x_attribute)))

    def update_defender(self, team, dt):
        is_blue = team == 'Blue'
        actor = 'blue_defender' if is_blue else 'red_defender'
        x_attribute = 'blue_defender_x' if is_blue else 'red_defender_x'
        y_attribute = 'blue_defender_y' if is_blue else 'red_defender_y'
        yaw_attribute = 'blue_defender_yaw' if is_blue else 'red_defender_yaw'
        speed_attribute = (
            'blue_defender_current_speed' if is_blue
            else 'red_defender_current_speed')
        x = getattr(self, x_attribute)
        y = getattr(self, y_attribute)
        yaw = getattr(self, yaw_attribute)
        if self.defender_ball_holder == team:
            self.hold_ball_with_defender(team)
            return

        target_ball_x, target_ball_y, sees_ball = self.known_ball_position(
            team, x, y, yaw)
        opponent_ball_holder = self.opponent_ball_holder_position(team)
        sees_opponent_with_ball = (
            opponent_ball_holder is not None and
            self.team_sees_opponent_ball_holder(team, opponent_ball_holder))
        if sees_opponent_with_ball:
            target_ball_x, target_ball_y = self.ball_x, self.ball_y
        is_own_team_in_possession = (
            (self.ball_controlled and self.ball_controlled_team == team) or
            self.goalie_ball_holder == team)
        pursues_ball = (
            target_ball_x is not None and not is_own_team_in_possession and
            self.closest_pursuer(team, target_ball_x, target_ball_y)
            == 'Defender')
        if not pursues_ball:
            target_ball_x, target_ball_y = None, None
        halfway = self.field_length * 0.5
        midfield_buffer = min(
            self.DEFENDER_MIDFIELD_BUFFER,
            halfway - self.robot_radius)
        own_half_ball = (
            target_ball_x is not None and
            (target_ball_x <= halfway if is_blue else target_ball_x >= halfway))
        attacking_goal_x = (
            self.opponent_goal_x if is_blue else self.blue_goal_line_x())
        if target_ball_x is not None:
            if own_half_ball:
                target_x, target_y = self.ball_target(
                    target_ball_x, target_ball_y, attacking_goal_x)
                target_y = target_ball_y
            else:
                target_x = (
                    halfway - midfield_buffer if is_blue else
                    halfway + midfield_buffer)
                target_y = target_ball_y
        else:
            opposing_team = 'Red' if is_blue else 'Blue'
            threat = self.team_robot_reports[opposing_team].get('Striker')
            own_goal_x = (
                self.blue_goal_line_x() if is_blue else self.opponent_goal_x)
            if threat is not None:
                threat_x, threat_y, _ = threat
                target_x = (own_goal_x + threat_x) * 0.5
                target_y = threat_y * 0.5
            else:
                target_x = halfway * 0.5 if is_blue else (
                    halfway + (self.field_length - halfway) * 0.5)
                target_y = 0.0
        if is_blue:
            target_x = min(max(target_x, self.robot_radius),
                           halfway - midfield_buffer)
        else:
            target_x = min(max(target_x, halfway + midfield_buffer),
                           self.field_length - self.robot_radius)
        target_y = min(
            max(target_y, -self.field_width * 0.5 + self.robot_radius),
            self.field_width * 0.5 - self.robot_radius)
        if not pursues_ball:
            striker_x, striker_y = (
                (self.robot_x, self.robot_y) if is_blue else
                (self.red_striker_x, self.red_striker_y))
            minimum_teammate_distance = 2.0 * self.robot_radius + 0.25
            if math.hypot(
                    target_x - striker_x, target_y - striker_y) < \
                    minimum_teammate_distance:
                attack_direction = 1.0 if is_blue else -1.0
                target_x = striker_x - (
                    attack_direction * minimum_teammate_distance)
                target_x = min(
                    max(target_x, self.robot_radius),
                    halfway - midfield_buffer if is_blue else
                    self.field_length - self.robot_radius)
                if not is_blue:
                    target_x = max(target_x, halfway + midfield_buffer)
                if math.hypot(
                        target_x - striker_x,
                        target_y - striker_y) < minimum_teammate_distance:
                    target_y = min(
                        max(striker_y + minimum_teammate_distance,
                            -self.field_width * 0.5 + self.robot_radius),
                        self.field_width * 0.5 - self.robot_radius)
        goalie_x, goalie_y = (
            (self.blue_goalie_x, self.blue_goalie_y) if is_blue else
            (self.opponent_x, self.opponent_y))
        minimum_goalie_distance = 2.0 * self.robot_radius + 0.25
        if math.hypot(target_x - goalie_x, target_y - goalie_y) < \
                minimum_goalie_distance:
            target_x = goalie_x + (
                (1.0 if is_blue else -1.0) * minimum_goalie_distance)
            target_x = min(
                max(target_x, self.robot_radius),
                halfway - self.robot_radius if is_blue else
                self.field_length - self.robot_radius)
            if not is_blue:
                target_x = max(target_x, halfway + self.robot_radius)
            target_y = goalie_y
        speed = getattr(self, speed_attribute)
        if sees_ball and pursues_ball:
            speed = min(
                speed + self.ROBOT_ACCELERATION * dt,
                self.MAX_ROBOT_SPEED)
        else:
            speed = max(
                min(self.robot_speed * self.UNSEEN_BALL_SPEED_FACTOR,
                    self.MAX_ROBOT_SPEED),
                speed - self.ROBOT_ACCELERATION * dt)
        setattr(self, speed_attribute, speed)
        dx, dy = target_x - x, target_y - y
        distance = math.hypot(dx, dy)
        if distance > 0.01:
            setattr(
                self, yaw_attribute,
                math.atan2(target_ball_y - y, target_ball_x - x)
                if target_ball_x is not None else
                math.atan2(dy, dx))
            step = min(speed * dt, distance)
            velocity_x, velocity_y = self.move_extra_robot(
                actor, dx * step / distance, dy * step / distance, dt)
            self.resolve_actor_ball_collision(
                actor, velocity_x, velocity_y)
        else:
            setattr(
                self, yaw_attribute,
                math.atan2(target_ball_y - y, target_ball_x - x)
                if target_ball_x is not None else yaw)

        if (self.ball_goal_side is None and
                self.goalie_ball_holder is None and
                self.defender_ball_holder is None and
                not self.kick_active and not self.ball_controlled and
                self.defender_touches_ball(team)):
            self.acquire_defender_ball(team)

    def red_striker_touches_ball(self):
        return math.hypot(
            self.ball_x - self.red_striker_x,
            self.ball_y - self.red_striker_y) <= (
                self.robot_radius + self.ball_radius + 0.02)

    def goalie_touches_ball(self, team):
        x, y = (
            (self.blue_goalie_x, self.blue_goalie_y)
            if team == 'Blue' else
            (self.opponent_x, self.opponent_y))
        return math.hypot(self.ball_x - x, self.ball_y - y) <= (
            self.robot_radius + self.ball_radius + 0.02)

    def pass_lane_clearance(
            self, team, target_x, target_y, start_x=None, start_y=None):
        start_x = self.ball_x if start_x is None else start_x
        start_y = self.ball_y if start_y is None else start_y
        direction_x = target_x - start_x
        direction_y = target_y - start_y
        length_squared = direction_x * direction_x + direction_y * direction_y
        if length_squared <= 1e-18:
            return -math.inf

        if team == 'Blue':
            blockers = (
                (self.red_striker_x, self.red_striker_y, self.robot_radius),
                (self.opponent_x, self.opponent_y, self.robot_radius),
                (self.red_defender_x, self.red_defender_y, self.robot_radius),
            )
        else:
            blockers = (
                (self.robot_x, self.robot_y, self.robot_radius),
                (self.blue_goalie_x, self.blue_goalie_y, self.robot_radius),
                (self.blue_defender_x, self.blue_defender_y,
                 self.robot_radius),
            )
        blockers += tuple(
            (obstacle['x'], obstacle['y'], obstacle['radius'])
            for obstacle in self.obstacles)
        clearance = math.inf
        for x, y, radius in blockers:
            projection = (
                (x - start_x) * direction_x +
                (y - start_y) * direction_y) / length_squared
            if not 0.0 < projection < 1.0:
                continue
            closest_x = start_x + projection * direction_x
            closest_y = start_y + projection * direction_y
            clearance = min(
                clearance,
                math.hypot(x - closest_x, y - closest_y) -
                radius - self.ball_radius)
        return clearance

    def pass_lane_is_clear(self, team, target_x, target_y):
        return self.pass_lane_clearance(
            team, target_x, target_y) > 0.0

    def ball_clearance_from_robots(
            self, team, role, ball_x, ball_y):
        if team == 'Blue':
            robots = (
                ('Striker', self.robot_x, self.robot_y),
                ('Goalie', self.blue_goalie_x, self.blue_goalie_y),
                ('Defender', self.blue_defender_x, self.blue_defender_y),
                ('Opponent Striker', self.red_striker_x, self.red_striker_y),
                ('Opponent Goalie', self.opponent_x, self.opponent_y),
                ('Opponent Defender', self.red_defender_x, self.red_defender_y),
            )
        else:
            robots = (
                ('Striker', self.red_striker_x, self.red_striker_y),
                ('Goalie', self.opponent_x, self.opponent_y),
                ('Defender', self.red_defender_x, self.red_defender_y),
                ('Opponent Striker', self.robot_x, self.robot_y),
                ('Opponent Goalie', self.blue_goalie_x, self.blue_goalie_y),
                ('Opponent Defender', self.blue_defender_x, self.blue_defender_y),
            )
        clearance = math.inf
        for blocker_role, blocker_x, blocker_y in robots:
            if blocker_role == role:
                continue
            clearance = min(
                clearance,
                math.hypot(ball_x - blocker_x, ball_y - blocker_y) -
                self.robot_radius - self.ball_radius)
        for obstacle in self.obstacles:
            clearance = min(
                clearance,
                math.hypot(
                    ball_x - obstacle['x'], ball_y - obstacle['y']) -
                obstacle['radius'] - self.ball_radius)
        return clearance

    def pass_ball_to_teammate(
            self, team, passer='Goalie', receiver='Striker'):
        team_positions = {
            'Blue': {
                'Striker': (self.robot_x, self.robot_y),
                'Goalie': (self.blue_goalie_x, self.blue_goalie_y),
                'Defender': (self.blue_defender_x, self.blue_defender_y),
            },
            'Red': {
                'Striker': (self.red_striker_x, self.red_striker_y),
                'Goalie': (self.opponent_x, self.opponent_y),
                'Defender': (self.red_defender_x, self.red_defender_y),
            },
        }
        try:
            teammate = team_positions[team][receiver]
            passer_position = team_positions[team][passer]
        except KeyError as error:
            raise rospy.ROSException(
                'Unsupported team role for a possession pass') from error
        if (self.last_pass is not None and
                self.last_pass[0] == team and
                {self.last_pass[1], self.last_pass[2]} ==
                {passer, receiver} and
                self.simulation_time - self.last_pass[3] <
                self.PASS_RETURN_COOLDOWN):
            return False
        if math.hypot(
                teammate[0] - passer_position[0],
                teammate[1] - passer_position[1]) < \
                self.MIN_TEAMMATE_PASS_DISTANCE:
            return False
        if not self.pass_lane_is_clear(team, teammate[0], teammate[1]):
            return False
        direction_x = teammate[0] - self.ball_x
        direction_y = teammate[1] - self.ball_y
        direction_length = math.hypot(direction_x, direction_y)
        if direction_length <= 1e-9:
            return False

        pass_speed = (
            self.kick_speed_kmh * self.GOALIE_PASS_SPEED_FACTOR / 3.6)
        self.ball_vx = direction_x * pass_speed / direction_length
        self.ball_vy = direction_y * pass_speed / direction_length
        self.kick_active = False
        self.ball_controlled = False
        self.last_pass = (team, passer, receiver, self.simulation_time)
        if passer == 'Goalie':
            self.goalie_ball_holder = None
            self.goalie_ball_hold_offset = None
        elif passer == 'Defender':
            self.defender_ball_holder = None
            self.defender_ball_hold_offset = None
            self.defender_carry_distance = 0.0
        elif passer == 'Striker':
            self.ball_controlled = False
            self.ball_controlled_team = None
            self.striker_carry_distance = 0.0
        rospy.loginfo(
            '%s %s passes to its %s at %.1f km/h',
            team, passer, receiver,
            self.kick_speed_kmh * self.GOALIE_PASS_SPEED_FACTOR)
        return True

    def move_ball_carrier_to_pass_lane(
            self, team, role, receiver, dt, distance_attribute):
        x_attribute, y_attribute, yaw_attribute = {
            ('Blue', 'Striker'): ('robot_x', 'robot_y', 'robot_yaw'),
            ('Red', 'Striker'): (
                'red_striker_x', 'red_striker_y', 'red_striker_yaw'),
            ('Blue', 'Defender'): (
                'blue_defender_x', 'blue_defender_y', 'blue_defender_yaw'),
            ('Red', 'Defender'): (
                'red_defender_x', 'red_defender_y', 'red_defender_yaw'),
        }[(team, role)]
        x = getattr(self, x_attribute)
        goal_x = (
            self.opponent_goal_x if team == 'Blue'
            else self.blue_goal_line_x())
        facing_yaw = 0.0 if goal_x >= x else math.pi
        forward_x, forward_y = math.cos(facing_yaw), math.sin(facing_yaw)
        setattr(self, yaw_attribute, facing_yaw)

        carried = getattr(self, distance_attribute)
        teammate = self.teammate_positions(team, role).get(receiver)
        if teammate is None:
            fallback_positions = {
                'Striker': {
                    'Blue': (self.robot_x, self.robot_y),
                    'Red': (self.red_striker_x, self.red_striker_y),
                },
                'Defender': {
                    'Blue': (self.blue_defender_x, self.blue_defender_y),
                    'Red': (self.red_defender_x, self.red_defender_y),
                },
            }
            teammate = fallback_positions[receiver][team]

        lateral_x, lateral_y = -forward_y, forward_x
        backward_boundary = (
            self.robot_radius if team == 'Blue'
            else self.field_length - self.robot_radius)
        available_to_boundary = (
            x - backward_boundary if team == 'Blue'
            else backward_boundary - x)
        backward_remaining = min(
            max(0.0, self.MAX_BACKWARD_CARRY_DISTANCE - carried),
            max(0.0, available_to_boundary))
        step = self.MAX_ROBOT_SPEED * dt
        if step <= 0.0:
            return False
        separation = self.robot_radius + self.ball_radius
        current_ball_clearance = self.ball_clearance_from_robots(
            team, role, self.ball_x, self.ball_y)
        ball_is_blocked = current_ball_clearance <= 0.05

        def find_candidate(angles, retreat=False):
            candidates = []
            for angle in angles:
                direction_x = (
                    forward_x * math.cos(angle) +
                    lateral_x * math.sin(angle))
                direction_y = (
                    forward_y * math.cos(angle) +
                    lateral_y * math.sin(angle))
                delta_x, delta_y = self.collision_avoiding_step(
                    x, getattr(self, y_attribute),
                    direction_x * step, direction_y * step)
                forward_progress = (
                    delta_x * forward_x + delta_y * forward_y)
                if retreat:
                    if forward_progress >= -1e-9:
                        continue
                    if -forward_progress > backward_remaining:
                        scale = backward_remaining / -forward_progress
                        delta_x *= scale
                        delta_y *= scale
                elif forward_progress < -1e-9:
                    continue

                candidate_x = min(
                    max(x + delta_x, self.robot_radius),
                    self.field_length - self.robot_radius)
                candidate_y = min(
                    max(getattr(self, y_attribute) + delta_y,
                        -self.field_width * 0.5 + self.robot_radius),
                    self.field_width * 0.5 - self.robot_radius)
                if role == 'Defender':
                    halfway = self.field_length * 0.5
                    if team == 'Blue':
                        candidate_x = min(
                            candidate_x, halfway - self.robot_radius)
                    else:
                        candidate_x = max(
                            candidate_x, halfway + self.robot_radius)
                actual_x = candidate_x - x
                actual_y = candidate_y - getattr(self, y_attribute)
                if math.hypot(actual_x, actual_y) <= 1e-9:
                    continue
                actual_forward_progress = (
                    actual_x * forward_x + actual_y * forward_y)
                if retreat:
                    if actual_forward_progress >= -1e-9:
                        continue
                    if -actual_forward_progress > backward_remaining:
                        scale = backward_remaining / -actual_forward_progress
                        candidate_x = x + actual_x * scale
                        candidate_y = (
                            getattr(self, y_attribute) + actual_y * scale)
                        actual_forward_progress = -backward_remaining
                elif actual_forward_progress < -1e-9:
                    continue

                ball_x = candidate_x + forward_x * separation
                ball_y = candidate_y + forward_y * separation
                clearance = self.pass_lane_clearance(
                    team, teammate[0], teammate[1], ball_x, ball_y)
                ball_clearance = self.ball_clearance_from_robots(
                    team, role, ball_x, ball_y)
                candidates.append((
                    clearance, ball_clearance, actual_forward_progress,
                    candidate_x, candidate_y))
            return candidates

        candidates = find_candidate((
            0.0, math.pi / 4.0, -math.pi / 4.0,
            math.pi / 2.0, -math.pi / 2.0))
        if ball_is_blocked:
            candidates.extend(find_candidate((
                math.pi / 2.0, -math.pi / 2.0)))
            if backward_remaining > 1e-9:
                candidates.extend(find_candidate((
                    math.pi, 3.0 * math.pi / 4.0,
                    -3.0 * math.pi / 4.0), retreat=True))
        elif not candidates and backward_remaining > 1e-9:
            candidates = find_candidate((
                math.pi, 3.0 * math.pi / 4.0, -3.0 * math.pi / 4.0),
                retreat=True)

        if not candidates:
            return False
        if ball_is_blocked:
            _, _, _, candidate_x, candidate_y = max(
                candidates, key=lambda candidate: (
                    candidate[1], candidate[0],
                    -abs(candidate[2])))
        else:
            _, _, _, candidate_x, candidate_y = max(
                candidates, key=lambda candidate: (
                    candidate[0], candidate[2]))
        delta_x = candidate_x - x
        delta_y = candidate_y - getattr(self, y_attribute)
        setattr(self, x_attribute, candidate_x)
        setattr(self, y_attribute, candidate_y)
        moved_x, moved_y = delta_x, delta_y
        moved_backward = max(
            0.0, -(moved_x * forward_x + moved_y * forward_y))
        setattr(
            self, distance_attribute,
            min(self.MAX_BACKWARD_CARRY_DISTANCE, carried + moved_backward))
        x = getattr(self, x_attribute)
        y = getattr(self, y_attribute)
        self.ball_x = x + forward_x * separation
        self.ball_y = y + forward_y * separation
        self.ball_vx = 0.0
        self.ball_vy = 0.0
        return math.hypot(delta_x, delta_y) > 1e-9

    def defender_touches_ball(self, team):
        x, y = (
            (self.blue_defender_x, self.blue_defender_y)
            if team == 'Blue' else
            (self.red_defender_x, self.red_defender_y))
        return math.hypot(self.ball_x - x, self.ball_y - y) <= (
            self.robot_radius + self.ball_radius + 0.02)

    def acquire_defender_ball(self, team):
        x = (
            self.blue_defender_x if team == 'Blue'
            else self.red_defender_x)
        goal_x = (
            self.opponent_goal_x if team == 'Blue'
            else self.blue_goal_line_x())
        yaw = 0.0 if goal_x >= x else math.pi
        if team == 'Blue':
            self.blue_defender_yaw = yaw
        else:
            self.red_defender_yaw = yaw
        separation = self.robot_radius + self.ball_radius
        self.defender_ball_hold_offset = (
            math.cos(yaw) * separation, math.sin(yaw) * separation)
        self.defender_ball_holder = team
        self.defender_carry_distance = 0.0
        self.kick_active = False
        self.ball_controlled = False
        self.ball_controlled_team = None
        self.striker_carry_distance = 0.0
        self.ball_vx = 0.0
        self.ball_vy = 0.0
        self.hold_ball_with_defender(team)
        rospy.loginfo('%s Defender gained control of the ball', team)

    def hold_ball_with_defender(self, team):
        x, y = (
            (self.blue_defender_x, self.blue_defender_y)
            if team == 'Blue' else
            (self.red_defender_x, self.red_defender_y))
        if self.defender_ball_hold_offset is None:
            self.acquire_defender_ball(team)
            return
        self.ball_x = x + self.defender_ball_hold_offset[0]
        self.ball_y = y + self.defender_ball_hold_offset[1]
        self.ball_vx = 0.0
        self.ball_vy = 0.0

    def update_defender_ball_possession(self, dt):
        team = self.defender_ball_holder
        if team is None:
            return
        fall_time = (
            self.blue_defender_fall_time if team == 'Blue'
            else self.red_defender_fall_time)
        if fall_time > 0.0:
            self.defender_ball_holder = None
            self.defender_ball_hold_offset = None
            self.defender_carry_distance = 0.0
            return
        self.hold_ball_with_defender(team)
        if self.pass_ball_to_teammate(
                team, passer='Defender', receiver='Striker'):
            return
        self.move_ball_carrier_to_pass_lane(
            team, 'Defender', 'Striker', dt, 'defender_carry_distance')

    def hold_ball_with_goalie(self, team):
        x, y = (
            (self.blue_goalie_x, self.blue_goalie_y)
            if team == 'Blue' else
            (self.opponent_x, self.opponent_y))
        if self.goalie_ball_holder != team:
            dx = self.ball_x - x
            dy = self.ball_y - y
            distance = math.hypot(dx, dy)
            if distance <= 1e-9:
                yaw = (
                    self.blue_goalie_yaw if team == 'Blue'
                    else self.opponent_yaw)
                dx, dy = math.cos(yaw), math.sin(yaw)
            else:
                dx /= distance
                dy /= distance
            separation = self.robot_radius + self.ball_radius
            self.goalie_ball_hold_offset = (
                dx * separation, dy * separation)
            self.goalie_ball_holder = team
            self.kick_active = False
            self.ball_controlled = False
            rospy.loginfo(
                '%s Goalie holds the ball while the pass lane is blocked',
                team)

        self.ball_x = x + self.goalie_ball_hold_offset[0]
        self.ball_y = y + self.goalie_ball_hold_offset[1]
        self.ball_vx = 0.0
        self.ball_vy = 0.0

    def update_goalie_ball_possession(self, dt):
        if (self.defender_ball_holder is not None or self.ball_controlled):
            return
        goalies = (
            ('Blue', 'blue_goalie_ball_contact_elapsed',
             self.blue_goalie_fall_time),
            ('Red', 'red_goalie_ball_contact_elapsed',
             self.opponent_fall_time),
        )
        for team, timer_attribute, fall_time in goalies:
            if fall_time > 0.0:
                setattr(self, timer_attribute, 0.0)
                if self.goalie_ball_holder == team:
                    self.goalie_ball_holder = None
                    self.goalie_ball_hold_offset = None
                continue

            if self.goalie_ball_holder == team:
                teammate = self.teammate_positions(
                    team, 'Goalie').get('Striker')
                if teammate is None:
                    teammate = (
                        (self.robot_x, self.robot_y) if team == 'Blue' else
                        (self.red_striker_x, self.red_striker_y))
                if (self.pass_lane_is_clear(
                        team, teammate[0], teammate[1]) and
                        self.pass_ball_to_teammate(team)):
                    setattr(self, timer_attribute, 0.0)
                    continue
                self.hold_ball_with_goalie(team)
                continue

            if (self.ball_goal_side is not None or
                    not self.goalie_touches_ball(team)):
                setattr(self, timer_attribute, 0.0)
                continue

            elapsed = getattr(self, timer_attribute) + dt
            setattr(self, timer_attribute, elapsed)
            if elapsed <= self.GOALIE_BALL_CONTACT_DURATION:
                continue
            if not self.pass_ball_to_teammate(team):
                self.hold_ball_with_goalie(team)
            else:
                setattr(self, timer_attribute, 0.0)
                break

    def move_extra_robot(self, actor, delta_x, delta_y, dt):
        if actor == 'blue_goalie':
            x_attribute, y_attribute = 'blue_goalie_x', 'blue_goalie_y'
        elif actor == 'red_striker':
            x_attribute, y_attribute = 'red_striker_x', 'red_striker_y'
        elif actor == 'blue_defender':
            x_attribute, y_attribute = 'blue_defender_x', 'blue_defender_y'
        else:
            x_attribute, y_attribute = 'red_defender_x', 'red_defender_y'
        old_x = getattr(self, x_attribute)
        old_y = getattr(self, y_attribute)
        delta_x, delta_y = self.collision_avoiding_step(
            old_x, old_y, delta_x, delta_y)
        setattr(self, x_attribute, old_x + delta_x)
        setattr(self, y_attribute, old_y + delta_y)
        if actor == 'blue_goalie':
            self.constrain_blue_goalkeeper_to_area()
        elif actor == 'blue_defender' or actor == 'red_defender':
            self.constrain_defenders_to_own_half()
        else:
            setattr(
                self, x_attribute,
                min(max(getattr(self, x_attribute), self.robot_radius),
                    self.field_length - self.robot_radius))
            setattr(
                self, y_attribute,
                min(max(getattr(self, y_attribute),
                        -self.field_width * 0.5 + self.robot_radius),
                    self.field_width * 0.5 - self.robot_radius))
        return (
            (getattr(self, x_attribute) - old_x) / dt,
            (getattr(self, y_attribute) - old_y) / dt)

    def update_strikers_ball_contact(self, dt):
        if (self.ball_goal_side is not None or
                self.defender_ball_holder is not None or
                self.goalie_ball_holder is not None or
                self.kick_active or self.ball_controlled or
                self.robot_fall_time > 0.0 or
                math.hypot(
                    self.robot_x - self.red_striker_x,
                    self.robot_y - self.red_striker_y) >
                self.STRIKER_PASS_DISTANCE or
                not self.robot_touches_ball() or
                not self.red_striker_touches_ball()):
            self.both_strikers_ball_contact_elapsed = 0.0
            return

        self.both_strikers_ball_contact_elapsed += dt
        if self.both_strikers_ball_contact_elapsed < \
                self.BOTH_STRIKERS_BALL_CONTACT_DURATION:
            return

        direction_x = self.ball_x - self.red_striker_x
        direction_y = self.ball_y - self.red_striker_y
        distance = math.hypot(direction_x, direction_y)
        if distance == 0.0:
            angle = self.random_generator.uniform(-math.pi, math.pi)
            direction_x, direction_y = math.cos(angle), math.sin(angle)
        else:
            direction_x /= distance
            direction_y /= distance

        self.kick_direction = (direction_x, direction_y)
        self.kick_elapsed = 0.0
        self.kick_active = True
        self.ball_controlled = False
        self.kick_target_y = self.ball_y + direction_y * self.field_length
        self.set_robot_in_contact()
        self.both_strikers_ball_contact_elapsed = 0.0
        rospy.loginfo(
            'Blue Striker kicks the ball away after prolonged contact')




    # Team-level dispatch keeps each side's robots and shared state together.





    # Game rules: field boundaries, scoring, and goal restart state.





    # Simulation loop and visualization.
    def update(self, event):
        if self.last_update is None:
            self.last_update = event.current_real
            return
        dt = min((event.current_real - self.last_update).to_sec(), 0.1)
        self.last_update = event.current_real
        if dt <= 0.0:
            return
        self.simulation_time += dt
        self.robot_fall_time = max(0.0, self.robot_fall_time - dt)
        self.opponent_fall_time = max(0.0, self.opponent_fall_time - dt)
        self.blue_goalie_fall_time = max(
            0.0, self.blue_goalie_fall_time - dt)
        self.red_striker_fall_time = max(
            0.0, self.red_striker_fall_time - dt)
        self.blue_defender_fall_time = max(
            0.0, self.blue_defender_fall_time - dt)
        self.red_defender_fall_time = max(
            0.0, self.red_defender_fall_time - dt)
        if self.robot_fall_time < 1e-9:
            self.robot_fall_time = 0.0
        if self.opponent_fall_time < 1e-9:
            self.opponent_fall_time = 0.0

        if self.ball_goal_side is not None:
            self.update_ball_in_goal(dt)
        elif self.defender_ball_holder is not None:
            self.hold_ball_with_defender(self.defender_ball_holder)
            self.update_team_ball_reports()
            self.update_blue_team(dt)
            self.update_red_team(dt)
            self.hold_ball_with_defender(self.defender_ball_holder)
        elif self.goalie_ball_holder is not None:
            self.hold_ball_with_goalie(self.goalie_ball_holder)
            self.update_team_ball_reports()
            self.update_blue_team(
                dt, update_striker=self.goalie_ball_holder != 'Blue')
            self.update_red_team(
                dt, update_striker=self.goalie_ball_holder != 'Red')
            self.hold_ball_with_goalie(self.goalie_ball_holder)
        elif self.kick_active:
            self.update_kick(dt)
            self.update_team_ball_reports()
            self.update_blue_team(dt, update_striker=False)
            self.update_red_team(dt, update_striker=False)
        elif (self.ball_controlled and
              (self.robot_fall_time if self.ball_controlled_team == 'Blue'
               else self.red_striker_fall_time) == 0.0):
            self.update_control(dt)
            self.update_team_ball_reports()
            self.update_blue_team(dt, update_striker=False)
            self.update_red_team(dt, update_striker=False)
        else:
            if not self.update_ball(dt):
                self.update_team_ball_reports()
                self.update_blue_team(dt)
                self.update_red_team(
                    dt, update_striker=not self.kick_active and
                    not self.ball_controlled)
        if self.resolve_robot_collision():
            self.constrain_goalkeeper_to_area()
            collided = self.last_collision_actors
            fall_timers = {
                'Blue Striker': 'robot_fall_time',
                'Blue Goalie': 'blue_goalie_fall_time',
                'Red Striker': 'red_striker_fall_time',
                'Red Goalie': 'opponent_fall_time',
                'Blue Defender': 'blue_defender_fall_time',
                'Red Defender': 'red_defender_fall_time',
            }
            for actor in collided:
                setattr(
                    self, fall_timers[actor], self.ROBOT_FALL_DURATION)

            if (self.ball_controlled_team == 'Blue' and
                    'Blue Striker' in collided) or (
                    self.ball_controlled_team == 'Red' and
                    'Red Striker' in collided):
                self.ball_controlled = False
                self.ball_controlled_team = None
                self.striker_carry_distance = 0.0
            if self.goalie_ball_holder == 'Blue' and 'Blue Goalie' in collided:
                self.goalie_ball_holder = None
                self.goalie_ball_hold_offset = None
                self.blue_goalie_ball_contact_elapsed = 0.0
            elif self.goalie_ball_holder == 'Red' and 'Red Goalie' in collided:
                self.goalie_ball_holder = None
                self.goalie_ball_hold_offset = None
                self.red_goalie_ball_contact_elapsed = 0.0
            if ((self.defender_ball_holder == 'Blue' and
                 'Blue Defender' in collided) or
                    (self.defender_ball_holder == 'Red' and
                     'Red Defender' in collided)):
                self.defender_ball_holder = None
                self.defender_ball_hold_offset = None
                self.defender_carry_distance = 0.0
            rospy.logwarn(
                '%s fell after a robot collision; ball control released',
                ' and '.join(sorted(collided)))
        self.update_strikers_ball_contact(dt)
        self.update_goalie_ball_possession(dt)
        self.update_defender_ball_possession(dt)
        if self.goalie_ball_holder is not None:
            self.hold_ball_with_goalie(self.goalie_ball_holder)
        elif self.defender_ball_holder is not None:
            self.hold_ball_with_defender(self.defender_ball_holder)
        self.update_robot_detection()
        self.publish_markers()

    def append_vision_markers(self, markers, stamp, marker_id, x, y, yaw,
                              vision_range, vision_aperture, color):
        fill = Marker()
        fill.header.frame_id = self.frame_id
        fill.header.stamp = stamp
        fill.ns = 'simulation'
        fill.id = marker_id
        fill.type = Marker.TRIANGLE_LIST
        fill.action = Marker.ADD
        fill.pose.orientation.w = 1.0
        fill.color = ColorRGBA(color[0], color[1], color[2], 0.05)

        outline = Marker()
        outline.header.frame_id = self.frame_id
        outline.header.stamp = stamp
        outline.ns = 'simulation'
        outline.id = marker_id + 1
        outline.type = Marker.LINE_LIST
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.025
        outline.color = ColorRGBA(color[0], color[1], color[2], 0.3)

        for index in range(self.VISION_SEGMENTS):
            angle1 = (
                yaw - vision_aperture * 0.5 +
                vision_aperture * index / self.VISION_SEGMENTS)
            angle2 = (
                yaw - vision_aperture * 0.5 +
                vision_aperture * (index + 1) / self.VISION_SEGMENTS)
            origin = Point(x=x, y=y, z=0.006)
            point1 = Point(
                x=x + vision_range * math.cos(angle1),
                y=y + vision_range * math.sin(angle1), z=0.006)
            point2 = Point(
                x=x + vision_range * math.cos(angle2),
                y=y + vision_range * math.sin(angle2), z=0.006)
            fill.points.extend([origin, point1, point2])
            if index == 0:
                outline.points.extend([
                    Point(x=x, y=y, z=0.008),
                    Point(x=point1.x, y=point1.y, z=0.008)])
            outline.points.extend([
                Point(x=point1.x, y=point1.y, z=0.008),
                Point(x=point2.x, y=point2.y, z=0.008)])
            if index == self.VISION_SEGMENTS - 1:
                outline.points.extend([
                    Point(x=point2.x, y=point2.y, z=0.008),
                    Point(x=x, y=y, z=0.008)])

        markers.markers.extend([fill, outline])

    def publish_markers(self):
        stamp = rospy.Time.now()
        markers = MarkerArray()

        ball = Marker()
        ball.header.frame_id = self.frame_id
        ball.header.stamp = stamp
        ball.ns = 'simulation'
        ball.id = 0
        ball.type = Marker.SPHERE
        ball.action = Marker.ADD
        ball.pose.position = Point(
            x=self.ball_x, y=self.ball_y, z=self.ball_radius)
        ball.pose.orientation.w = 1.0
        ball.scale.x = ball.scale.y = ball.scale.z = self.ball_radius * 2.0
        ball.color = (ColorRGBA(1.0, 0.75, 0.1, 1.0)
                      if self.ball_controlled else
                      ColorRGBA(0.95, 0.95, 0.95, 1.0))
        markers.markers.append(ball)

        robot = Marker()
        robot.header.frame_id = self.frame_id
        robot.header.stamp = stamp
        robot.ns = 'simulation'
        robot.id = 1
        robot.type = Marker.CYLINDER
        robot.action = Marker.ADD
        robot.pose.position = Point(
            x=self.robot_x, y=self.robot_y, z=self.robot_height * 0.5)
        robot.pose.orientation.z = math.sin(self.robot_yaw * 0.5)
        robot.pose.orientation.w = math.cos(self.robot_yaw * 0.5)
        robot.scale.x = robot.scale.y = self.robot_radius * 2.0
        robot.scale.z = self.robot_height
        robot.color = (
            ColorRGBA(0.45, 0.25, 0.12, 1.0)
            if self.robot_fall_time > 0.0 else
            ColorRGBA(1.0, 0.45, 0.05, 1.0)
            if self.kick_active else
            ColorRGBA(0.4, 0.75, 1.0, 1.0))
        markers.markers.append(robot)

        opponent = Marker()
        opponent.header.frame_id = self.frame_id
        opponent.header.stamp = stamp
        opponent.ns = 'simulation'
        opponent.id = 6
        opponent.type = Marker.CYLINDER
        opponent.action = Marker.ADD
        opponent.pose.position = Point(
            x=self.opponent_x, y=self.opponent_y,
            z=self.robot_height * 0.5)
        opponent.pose.orientation.z = math.sin(self.opponent_yaw * 0.5)
        opponent.pose.orientation.w = math.cos(self.opponent_yaw * 0.5)
        opponent.scale.x = opponent.scale.y = self.robot_radius * 2.0
        opponent.scale.z = self.robot_height
        opponent.color = (
            ColorRGBA(1.0, 0.0, 0.0, 1.0)
            if self.opponent_fall_time > 0.0 else
            ColorRGBA(0.9, 0.12, 0.12, 1.0))
        markers.markers.append(opponent)

        for marker_id, x, y, yaw, fall_time, color in (
                (16, self.blue_goalie_x, self.blue_goalie_y,
                 self.blue_goalie_yaw, self.blue_goalie_fall_time,
                 ColorRGBA(0.15, 0.45, 0.9, 1.0)),
                (17, self.red_striker_x, self.red_striker_y,
                 self.red_striker_yaw, self.red_striker_fall_time,
                 ColorRGBA(0.95, 0.35, 0.45, 1.0)),
                (26, self.blue_defender_x, self.blue_defender_y,
                 self.blue_defender_yaw, self.blue_defender_fall_time,
                 ColorRGBA(0.15, 0.55, 0.95, 1.0)),
                (27, self.red_defender_x, self.red_defender_y,
                 self.red_defender_yaw, self.red_defender_fall_time,
                 ColorRGBA(0.95, 0.25, 0.35, 1.0))):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = 'simulation'
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position = Point(
                x=x, y=y, z=self.robot_height * 0.5)
            marker.pose.orientation.z = math.sin(yaw * 0.5)
            marker.pose.orientation.w = math.cos(yaw * 0.5)
            marker.scale.x = marker.scale.y = self.robot_radius * 2.0
            marker.scale.z = self.robot_height
            marker.color = (
                ColorRGBA(0.45, 0.25, 0.12, 1.0)
                if fall_time > 0.0 else color)
            markers.markers.append(marker)

        for marker_id, text, x, y, color in (
                (14, 'Blue Striker', self.robot_x, self.robot_y,
                 ColorRGBA(0.4, 0.75, 1.0, 1.0)),
                (15, 'Red Goalie', self.opponent_x, self.opponent_y,
                 ColorRGBA(0.9, 0.12, 0.12, 1.0)),
                (20, 'Blue Goalie', self.blue_goalie_x, self.blue_goalie_y,
                 ColorRGBA(0.15, 0.45, 0.9, 1.0)),
                (21, 'Red Striker', self.red_striker_x, self.red_striker_y,
                 ColorRGBA(0.95, 0.35, 0.45, 1.0)),
                (28, 'Blue Defender', self.blue_defender_x,
                 self.blue_defender_y, ColorRGBA(0.15, 0.55, 0.95, 1.0)),
                (29, 'Red Defender', self.red_defender_x,
                 self.red_defender_y, ColorRGBA(0.95, 0.25, 0.35, 1.0))):
            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = 'simulation'
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(
                x=x, y=y, z=self.robot_height + 0.12)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.16
            label.color = color
            label.text = text
            markers.markers.append(label)

        for marker_id in range(10, 14):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = 'simulation'
            marker.id = marker_id
            marker.action = Marker.DELETE
            markers.markers.append(marker)

        scoreboard = Marker()
        scoreboard.header.frame_id = self.frame_id
        scoreboard.header.stamp = stamp
        scoreboard.ns = 'simulation'
        scoreboard.id = 4
        scoreboard.type = Marker.TEXT_VIEW_FACING
        scoreboard.action = Marker.ADD
        scoreboard.pose.position = Point(
            x=self.field_length * 0.5,
            y=self.field_width * 0.5 + 0.45,
            z=0.6)
        scoreboard.pose.orientation.w = 1.0
        scoreboard.scale.z = 0.3
        scoreboard.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        scoreboard.text = 'GOALS  Left: {}    Right: {}'.format(
            self.left_score, self.right_score)
        markers.markers.append(scoreboard)

        if self.ball_controlled or self.kick_active:
            shot_line = Marker()
            shot_line.header.frame_id = self.frame_id
            shot_line.header.stamp = stamp
            shot_line.ns = 'simulation'
            shot_line.id = 5
            shot_line.type = Marker.LINE_STRIP
            shot_line.action = Marker.ADD
            shot_line.pose.orientation.w = 1.0
            shot_line.scale.x = 0.04
            shot_line.color = ColorRGBA(1.0, 0.8, 0.1, 0.95)
            shot_line.points = [
                Point(x=self.robot_x, y=self.robot_y, z=0.02),
                Point(x=self.ball_x, y=self.ball_y, z=0.02),
                Point(x=self.opponent_goal_x, y=self.kick_target_y, z=0.02),
            ]
            markers.markers.append(shot_line)

        for obstacle_index, obstacle in enumerate(self.obstacles, start=10):
            obstacle_marker = Marker()
            obstacle_marker.header.frame_id = self.frame_id
            obstacle_marker.header.stamp = stamp
            obstacle_marker.ns = 'obstacles'
            obstacle_marker.id = obstacle_index
            obstacle_marker.type = Marker.CYLINDER
            obstacle_marker.action = Marker.ADD
            obstacle_marker.pose.position = Point(
                x=obstacle['x'], y=obstacle['y'], z=0.15)
            obstacle_marker.pose.orientation.w = 1.0
            obstacle_marker.scale.x = obstacle['radius'] * 2.0
            obstacle_marker.scale.y = obstacle['radius'] * 2.0
            obstacle_marker.scale.z = 0.3
            obstacle_marker.color = ColorRGBA(0.9, 0.2, 0.15, 1.0)
            markers.markers.append(obstacle_marker)

        vision_fill = Marker()
        vision_fill.header.frame_id = self.frame_id
        vision_fill.header.stamp = stamp
        vision_fill.ns = 'simulation'
        vision_fill.id = 2
        vision_fill.type = Marker.TRIANGLE_LIST
        vision_fill.action = Marker.ADD
        vision_fill.pose.orientation.w = 1.0
        vision_fill.color = ColorRGBA(0.1, 0.85, 0.35, 0.16)
        for index in range(self.VISION_SEGMENTS):
            angle1 = (self.robot_yaw - self.vision_aperture * 0.5 +
                      self.vision_aperture * index / self.VISION_SEGMENTS)
            angle2 = (self.robot_yaw - self.vision_aperture * 0.5 +
                      self.vision_aperture * (index + 1) /
                      self.VISION_SEGMENTS)
            vision_fill.points.extend([
                Point(x=self.robot_x, y=self.robot_y, z=0.006),
                Point(
                    x=self.robot_x + self.vision_range * math.cos(angle1),
                    y=self.robot_y + self.vision_range * math.sin(angle1),
                    z=0.006),
                Point(
                    x=self.robot_x + self.vision_range * math.cos(angle2),
                    y=self.robot_y + self.vision_range * math.sin(angle2),
                    z=0.006),
            ])
        markers.markers.append(vision_fill)

        vision_outline = Marker()
        vision_outline.header.frame_id = self.frame_id
        vision_outline.header.stamp = stamp
        vision_outline.ns = 'simulation'
        vision_outline.id = 3
        vision_outline.type = Marker.LINE_LIST
        vision_outline.action = Marker.ADD
        vision_outline.pose.orientation.w = 1.0
        vision_outline.scale.x = 0.025
        vision_outline.color = ColorRGBA(0.2, 1.0, 0.45, 0.9)
        for index in range(self.VISION_SEGMENTS):
            angle1 = (self.robot_yaw - self.vision_aperture * 0.5 +
                      self.vision_aperture * index / self.VISION_SEGMENTS)
            angle2 = (self.robot_yaw - self.vision_aperture * 0.5 +
                      self.vision_aperture * (index + 1) /
                      self.VISION_SEGMENTS)
            point1 = Point(
                x=self.robot_x + self.vision_range * math.cos(angle1),
                y=self.robot_y + self.vision_range * math.sin(angle1),
                z=0.008)
            point2 = Point(
                x=self.robot_x + self.vision_range * math.cos(angle2),
                y=self.robot_y + self.vision_range * math.sin(angle2),
                z=0.008)
            if index == 0:
                vision_outline.points.extend([
                    Point(x=self.robot_x, y=self.robot_y, z=0.008), point1])
            vision_outline.points.extend([point1, point2])
            if index == self.VISION_SEGMENTS - 1:
                vision_outline.points.extend([
                    point2, Point(x=self.robot_x, y=self.robot_y, z=0.008)])
        markers.markers.append(vision_outline)

        opponent_vision_fill = Marker()
        opponent_vision_fill.header.frame_id = self.frame_id
        opponent_vision_fill.header.stamp = stamp
        opponent_vision_fill.ns = 'simulation'
        opponent_vision_fill.id = 7
        opponent_vision_fill.type = Marker.TRIANGLE_LIST
        opponent_vision_fill.action = Marker.ADD
        opponent_vision_fill.pose.orientation.w = 1.0
        opponent_vision_fill.color = ColorRGBA(0.95, 0.12, 0.12, 0.05)
        for index in range(self.VISION_SEGMENTS):
            angle1 = (
                self.opponent_yaw - self.goalie_vision_aperture * 0.5 +
                self.goalie_vision_aperture * index / self.VISION_SEGMENTS)
            angle2 = (
                self.opponent_yaw - self.goalie_vision_aperture * 0.5 +
                self.goalie_vision_aperture * (index + 1) /
                self.VISION_SEGMENTS)
            opponent_vision_fill.points.extend([
                Point(x=self.opponent_x, y=self.opponent_y, z=0.006),
                Point(
                    x=self.opponent_x + self.goalie_vision_range * math.cos(angle1),
                    y=self.opponent_y + self.goalie_vision_range * math.sin(angle1),
                    z=0.006),
                Point(
                    x=self.opponent_x + self.goalie_vision_range * math.cos(angle2),
                    y=self.opponent_y + self.goalie_vision_range * math.sin(angle2),
                    z=0.006),
            ])
        markers.markers.append(opponent_vision_fill)

        opponent_vision_outline = Marker()
        opponent_vision_outline.header.frame_id = self.frame_id
        opponent_vision_outline.header.stamp = stamp
        opponent_vision_outline.ns = 'simulation'
        opponent_vision_outline.id = 8
        opponent_vision_outline.type = Marker.LINE_LIST
        opponent_vision_outline.action = Marker.ADD
        opponent_vision_outline.pose.orientation.w = 1.0
        opponent_vision_outline.scale.x = 0.025
        opponent_vision_outline.color = ColorRGBA(1.0, 0.15, 0.15, 0.3)
        for index in range(self.VISION_SEGMENTS):
            angle1 = (
                self.opponent_yaw - self.goalie_vision_aperture * 0.5 +
                self.goalie_vision_aperture * index / self.VISION_SEGMENTS)
            angle2 = (
                self.opponent_yaw - self.goalie_vision_aperture * 0.5 +
                self.goalie_vision_aperture * (index + 1) /
                self.VISION_SEGMENTS)
            point1 = Point(
                x=self.opponent_x + self.goalie_vision_range * math.cos(angle1),
                y=self.opponent_y + self.goalie_vision_range * math.sin(angle1),
                z=0.008)
            point2 = Point(
                x=self.opponent_x + self.goalie_vision_range * math.cos(angle2),
                y=self.opponent_y + self.goalie_vision_range * math.sin(angle2),
                z=0.008)
            if index == 0:
                opponent_vision_outline.points.extend([
                    Point(
                        x=self.opponent_x, y=self.opponent_y, z=0.008),
                    point1])
            opponent_vision_outline.points.extend([point1, point2])
            if index == self.VISION_SEGMENTS - 1:
                opponent_vision_outline.points.extend([
                    point2,
                    Point(
                        x=self.opponent_x, y=self.opponent_y, z=0.008)])
        markers.markers.append(opponent_vision_outline)

        self.append_vision_markers(
            markers, stamp, 22, self.blue_goalie_x, self.blue_goalie_y,
            self.blue_goalie_yaw, self.goalie_vision_range,
            self.goalie_vision_aperture, (0.15, 0.45, 0.9))
        self.append_vision_markers(
            markers, stamp, 24, self.red_striker_x, self.red_striker_y,
            self.red_striker_yaw, self.vision_range,
            self.vision_aperture, (0.95, 0.2, 0.35))
        self.append_vision_markers(
            markers, stamp, 30, self.blue_defender_x, self.blue_defender_y,
            self.blue_defender_yaw, self.vision_range,
            self.vision_aperture, (0.15, 0.55, 0.95))
        self.append_vision_markers(
            markers, stamp, 32, self.red_defender_x, self.red_defender_y,
            self.red_defender_yaw, self.vision_range,
            self.vision_aperture, (0.95, 0.25, 0.35))

        self.publisher.publish(markers)

if __name__ == '__main__':
    rospy.init_node('soccer_2d_sim')
    try:
        Soccer2DSimulation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
