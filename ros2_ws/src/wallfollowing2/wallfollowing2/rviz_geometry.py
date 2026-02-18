import numpy as np

from geometry_msgs.msg import Point as PointMessage
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from .circle import Point


RVIZ_TOPIC = "/wallfollowing_visualization"
RVIZ_FRAME = "car/chassis/ust10lx"
RVIZ_NAMESPACE = "wall_following"


def _normalize_color(color):
    if isinstance(color, ColorRGBA):
        return color
    if isinstance(color, tuple) and len(color) == 4:
        return ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
    raise TypeError("color must be ColorRGBA or RGBA tuple")


def _make_marker(clock, marker_id, color, line_width):
    message = Marker()
    message.header.frame_id = RVIZ_FRAME
    message.header.stamp = clock.now().to_msg()
    message.ns = RVIZ_NAMESPACE
    message.id = marker_id
    message.action = Marker.ADD
    message.type = Marker.LINE_STRIP
    message.pose.orientation.w = 1.0
    message.scale.x = line_width
    message.color = _normalize_color(color)
    return message


def _point_msg(x, y, z=0.0):
    point = PointMessage()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def show_line_in_rviz(publisher, clock, marker_id, points, color, line_width=0.02):
    message = _make_marker(clock, marker_id, color, line_width)
    if isinstance(points, np.ndarray):
        message.points = [
            _point_msg(points[i, 1], -points[i, 0], 0.0)
            for i in range(points.shape[0])
        ]
    elif isinstance(points, list):
        message.points = [
            _point_msg(point.y, -point.x, 0.0)
            for point in points
        ]
    else:
        raise TypeError(
            "points must be a numpy array or list of points, got "
            + str(type(points))
        )
    publisher.publish(message)


def show_circle_in_rviz(publisher, clock, circle, wall, marker_id):
    start_angle = circle.get_angle(Point(wall[0, 0], wall[0, 1]))
    end_angle = circle.get_angle(Point(wall[-1, 0], wall[-1, 1]))
    points = circle.create_array(start_angle, end_angle)
    show_line_in_rviz(
        publisher, clock, marker_id, points, color=(0.0, 1.0, 1.0, 1.0)
    )
