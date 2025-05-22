# 编码: utf-8

import freetype
import rospy
import copy
from geometry_msgs.msg import Pose, Quaternion

# 辅助类
class OutlineDecomposer:
    def __init__(self, num_samples_per_curve_segment):
        self.points_2d_font_units = []  # 字体单位中的(x,y)元组列表
        self._current_pen_pos = None
        self._num_samples = max(2, num_samples_per_curve_segment)
    def _add_point(self, p_tuple):
        epsilon = 1e-5 
        if not self.points_2d_font_units or \
           abs(self.points_2d_font_units[-1][0] - p_tuple[0]) > epsilon or \
           abs(self.points_2d_font_units[-1][1] - p_tuple[1]) > epsilon:
            self.points_2d_font_units.append(p_tuple)
    def move_to(self, a, _=None):
        self._current_pen_pos = (a.x, a.y)
        self._add_point(self._current_pen_pos) 
        return 0
    def line_to(self, a, _=None):
        if self._current_pen_pos is None:
            rospy.logwarn("line_to called without prior move_to establishing current position.")
            self._current_pen_pos = (a.x, a.y)
            self._add_point(self._current_pen_pos)
            return 0
        self._current_pen_pos = (a.x, a.y)
        self._add_point(self._current_pen_pos)
        return 0
    def conic_to(self, control, to, _=None):
        if self._current_pen_pos is None:
            rospy.logwarn("conic_to called without prior move_to establishing current position.")
            return -1 
        p0 = self._current_pen_pos
        p1_ctrl = (control.x, control.y)
        p2_end = (to.x, to.y)
        for i in range(1, self._num_samples):
            t = float(i) / (self._num_samples - 1)
            omt = 1.0 - t
            x = omt**2 * p0[0] + 2*omt*t * p1_ctrl[0] + t**2 * p2_end[0]
            y = omt**2 * p0[1] + 2*omt*t * p1_ctrl[1] + t**2 * p2_end[1]
            self._add_point((x, y))
        self._current_pen_pos = p2_end
        return 0
    def cubic_to(self, control1, control2, to, _=None):
        if self._current_pen_pos is None:
            rospy.logwarn("cubic_to called without prior move_to establishing current position.")
            return -1
        p0 = self._current_pen_pos
        p1_ctrl = (control1.x, control1.y)
        p2_ctrl = (control2.x, control2.y)
        p3_end = (to.x, to.y)
        for i in range(1, self._num_samples):
            t = float(i) / (self._num_samples - 1)
            omt = 1.0 - t
            x = omt**3 * p0[0] + \
                3*omt**2*t * p1_ctrl[0] + \
                3*omt*t**2 * p2_ctrl[0] + \
                t**3 * p3_end[0]
            y = omt**3 * p0[1] + \
                3*omt**2*t * p1_ctrl[1] + \
                3*omt*t**2 * p2_ctrl[1] + \
                t**3 * p3_end[1]
            self._add_point((x, y))
        self._current_pen_pos = p3_end
        return 0
# 使用字体文件生成字符路径点（YZ平面）
def generate_letter_waypoints_from_font(char_to_draw, font_path, desired_letter_height_m, num_samples_per_curve, center_pose, rospy_instance=None):
    """
    参数:
        char_to_draw (str): 要绘制的字符。
        font_path (str): .ttf 或 .otf 字体文件的绝对路径。
        desired_letter_height_m (float): 字母的目标高度，单位为米。
        num_samples_per_curve (int): 每个贝塞尔曲线段的样本数。
        center_pose (geometry_msgs.msg.Pose): 字母的中心姿态。
        rospy_instance: 如果在此函数内需要日志记录，则传入 rospy。
    返回:
        geometry_msgs.msg.Pose 列表: 路径点列表，失败时返回空列表。
    """
    log_info = rospy_instance.loginfo if rospy_instance else rospy.loginfo
    log_err = rospy_instance.logerr if rospy_instance else rospy.logerr
    log_warn = rospy_instance.logwarn if rospy_instance else rospy.logwarn
    waypoints = []
    if center_pose is None:
        log_err("center_pose cannot be None in generate_letter_waypoints_from_font.")
        return []
    base_x = center_pose.position.x
    offset_y_world = center_pose.position.y
    offset_z_world = center_pose.position.z
    fixed_orientation = center_pose.orientation
    try:
        face = freetype.Face(font_path)
    except freetype.FT_Exception as e:
        log_err(f"Failed to load font: {font_path}. Error: {e}")
        return []
    face.set_pixel_sizes(0, 64) 
    try:
        face.load_char(char_to_draw, freetype.FT_LOAD_NO_SCALE | freetype.FT_LOAD_NO_BITMAP)
    except freetype.FT_Exception as e:
        log_err(f"Failed to load glyph for char '{char_to_draw}'. Error: {e}")
        return []
    outline = face.glyph.outline
    decomposer = OutlineDecomposer(num_samples_per_curve_segment=num_samples_per_curve) 
    outline.decompose(decomposer, move_to=decomposer.move_to, line_to=decomposer.line_to, conic_to=decomposer.conic_to, cubic_to=decomposer.cubic_to)
    font_units_points = decomposer.points_2d_font_units
    if not font_units_points:
        log_warn(f"No points generated for char '{char_to_draw}'. It might be an empty glyph or decomposition issue.")
        return []
    min_font_x = min(p[0] for p in font_units_points)
    max_font_x = max(p[0] for p in font_units_points)
    min_font_y = min(p[1] for p in font_units_points)
    max_font_y = max(p[1] for p in font_units_points)
    font_width_fu = max_font_x - min_font_x
    font_height_fu = max_font_y - min_font_y
    if font_height_fu == 0:
        if font_width_fu == 0:
             log_warn(f"Character '{char_to_draw}' has zero width and height in font units.")
             scale_factor = 0.0
        else:
             scale_factor = desired_letter_height_m / font_width_fu if font_width_fu else 0.0
    else:
        scale_factor = desired_letter_height_m / font_height_fu
    center_font_x = (min_font_x + max_font_x) / 2.0
    center_font_y = (min_font_y + max_font_y) / 2.0
    for fx, fy in font_units_points:
        local_y = (fx - center_font_x) * scale_factor
        local_z = (fy - center_font_y) * scale_factor
        pose = Pose()
        pose.position.x = base_x
        pose.position.y = offset_y_world + local_y
        pose.position.z = offset_z_world + local_z
        pose.orientation = copy.deepcopy(fixed_orientation)
        waypoints.append(pose)
    log_info(f"Generated {len(waypoints)} waypoints for char '{char_to_draw}' with height {desired_letter_height_m:.2f}m (from helper).")
    return waypoints