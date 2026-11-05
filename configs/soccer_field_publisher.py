#!/usr/bin/env python3
import math
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class SoccerFieldPublisher:
    def __init__(self):
        rospy.init_node('soccer_field_publisher')
        self.frame_id = rospy.get_param('~frame_id', 'soccer_field_link')

        # === Pitch dimensions (Table 1) ===
        self.L = rospy.get_param('~pitch_length', 9.0)      # A
        self.W = rospy.get_param('~pitch_width', 6.0)       # B
        self.goal_depth = rospy.get_param('~goal_depth', 0.6)   # C
        self.goal_w = rospy.get_param('~goal_width', 2.6)       # D
        self.goal_h = rospy.get_param('~goal_height', 1.2)      # E
        self.ga_len = rospy.get_param('~goal_area_length', 1.0) # E/GA length
        self.ga_wid = rospy.get_param('~goal_area_width', 3.0)  # F
        self.pen_dist = rospy.get_param('~penalty_mark_distance', 1.5)  # G
        self.cc_dia = rospy.get_param('~centre_circle_diameter', 1.5)   # H
        self.pa_len = rospy.get_param('~penalty_area_length', 2.0)      # J
        self.pa_wid = rospy.get_param('~penalty_area_width', 5.0)       # K
        self.line_width = rospy.get_param('~line_width', 0.05)

        self.pub = rospy.Publisher('/soccer_field/markers', Marker, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_field)
        rospy.loginfo("Mini soccer field publisher started (frame: %s)" % self.frame_id)

    # ---------- helpers ----------
    def p(self, x, y, z=0.002):
        return Point(x=x, y=y, z=z)

    def base_marker(self, mid, mtype, color):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = "field_%d" % mid
        m.id = 0
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = self.line_width
        m.scale.y = self.line_width
        m.color = color
        return m

    def line_list(self, mid, pts, color):
        m = self.base_marker(mid, Marker.LINE_LIST, color)
        m.points = pts
        return m

    def circle_lines(self, cx, cy, radius, n=48):
        pts = []
        for i in range(n):
            a1 = 2 * math.pi * i / n
            a2 = 2 * math.pi * (i + 1) / n
            pts.append(self.p(cx + radius * math.cos(a1), cy + radius * math.sin(a1)))
            pts.append(self.p(cx + radius * math.cos(a2), cy + radius * math.sin(a2)))
        return pts

    def rect_lines(self, x_min, x_max, y_min, y_max):
        """Three sides of a box open toward the goal line."""
        return [
            self.p(x_min, y_min), self.p(x_max, y_min),
            self.p(x_max, y_min), self.p(x_max, y_max),
            self.p(x_max, y_max), self.p(x_min, y_max),
        ]

    # ---------- main drawing ----------
    def publish_field(self, event=None):
        L, W = self.L, self.W
        hl, hw = L / 2.0, W / 2.0
        white = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        markers = []

        # --- Outer boundary ---
        boundary = [
            self.p(0, -hw), self.p(L, -hw),
            self.p(L, -hw), self.p(L, hw),
            self.p(L, hw), self.p(0, hw),
            self.p(0, hw), self.p(0, -hw),
        ]
        markers.append(self.line_list(1, boundary, white))

        # --- Halfway line ---
        markers.append(self.line_list(2, [self.p(hl, -hw), self.p(hl, hw)], white))

        # --- Centre circle (diameter 1.5 -> radius 0.75) ---
        markers.append(self.line_list(3, self.circle_lines(hl, 0, self.cc_dia / 2.0), white))

        # --- Penalty areas (2m deep x 5m wide) ---
        pa = (self.rect_lines(0, self.pa_len, -self.pa_wid / 2.0, self.pa_wid / 2.0) +
              self.rect_lines(L, L - self.pa_len, -self.pa_wid / 2.0, self.pa_wid / 2.0))
        markers.append(self.line_list(4, pa, white))

        # --- Goal areas (1m deep x 3m wide) ---
        ga = (self.rect_lines(0, self.ga_len, -self.ga_wid / 2.0, self.ga_wid / 2.0) +
              self.rect_lines(L, L - self.ga_len, -self.ga_wid / 2.0, self.ga_wid / 2.0))
        markers.append(self.line_list(5, ga, white))

        # --- Penalty marks (1.5m from goal line) ---
        spots = self.base_marker(6, Marker.SPHERE_LIST, white)
        spots.scale.x = spots.scale.y = spots.scale.z = 0.1
        spots.points = [
            self.p(self.pen_dist, 0, 0.05),
            self.p(L - self.pen_dist, 0, 0.05),
        ]
        markers.append(spots)

        # --- Goals (2.6m wide x 1.2m high, 0.6m deep) ---
        gp = []
        for gx, sign in [(0, -1), (L, 1)]:
            back = gx + sign * self.goal_depth
            gp += [
                self.p(gx, -self.goal_w / 2), self.p(back, -self.goal_w / 2),
                self.p(back, -self.goal_w / 2), self.p(back, self.goal_w / 2),
                self.p(back, self.goal_w / 2), self.p(gx, self.goal_w / 2),
                self.p(gx, -self.goal_w / 2), self.p(gx, -self.goal_w / 2, self.goal_h + 0.002),
                self.p(gx, self.goal_w / 2), self.p(gx, self.goal_w / 2, self.goal_h + 0.002),
                self.p(gx, -self.goal_w / 2, self.goal_h + 0.002),
                self.p(gx, self.goal_w / 2, self.goal_h + 0.002),
            ]
        markers.append(self.line_list(7, gp, white))

        for m in markers:
            self.pub.publish(m)


if __name__ == '__main__':
    try:
        SoccerFieldPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass