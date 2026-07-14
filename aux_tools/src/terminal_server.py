# @file terminal_server.py
# @author HKUST Aerial Robotics Group
# @brief terminal server for testing
# @version 0.1
# @date 2019-02
# @copyright Copyright (c) 2019

"""基于 Pygame 的 ROS2 键盘控制与俯视可视化终端。

职责边界：本模块订阅仿真地图和控制轨迹、发布等价于手柄按键的 ``Joy`` 消息，
并把车辆/车道投影到二维窗口。它不计算行为决策、车辆控制量或安全轨迹，不能
作为规划结果评测器使用。
"""

# sys
import time
import sys
import shutil
import random
from math import *
from functools import reduce

# pygame
import pygame as pg
from pygame.locals import *
from pygame.math import Vector2
# ROS2
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

# msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from vehicle_msgs.msg import ArenaInfoDynamic
from vehicle_msgs.msg import ArenaInfoStatic
from vehicle_msgs.msg import State
from vehicle_msgs.msg import ControlSignal

# 当前终端只围绕 0 号自车建立显示坐标系，并允许选择任意车辆发送 HMI 指令。
ego_id = 0
agent_id = 0

# Pygame 画布尺寸以及世界坐标到像素坐标的缩放比例。
width = 800
height = 600
scale = 4.0
# 全局渲染状态由 ROS2 单线程回调和 50 Hz 定时器串行访问。
screen = pg.display.set_mode((width, height))
all_sprites = pg.sprite.Group()
vehicles = {}
recorded_ids = []
lane_pts = []
center_3dof = (0.0, 0.0, 0.0)
state_seq = []
has_arena_info_dynamic = False


class TerminalServerNode(Node):
    """连接 ROS2 消息与 Pygame 终端状态的轻量节点。"""

    def __init__(self):
        """创建 Joy 发布器、环境订阅器和 50 Hz 可视化定时器。"""
        super().__init__('key2joy')
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.create_subscription(ArenaInfoDynamic, "/arena_info_dynamic", self.process_arena_info_dynamic, 10)
        self.create_subscription(ArenaInfoStatic, "/arena_info_static", self.process_arena_info_static, 10)
        self.create_subscription(ControlSignal, "/ctrl/agent_0", self.process_control_signal, 10)
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)  # 50 Hz

    def timer_callback(self):
        """轮询输入事件、刷新车辆/车道画面并提交一帧显示。"""
        handle_keyboard_event(self.joy_pub)
        update_visualization()
        pg.display.update()

    def process_arena_info_dynamic(self, data):
        """更新车辆动态状态，并为首次出现的车辆创建显示精灵。

        Args:
            data: ``ArenaInfoDynamic`` 消息，包含当前所有车辆的运动状态。
        """
        global center_3dof, has_arena_info_dynamic
        for v in data.vehicle_set.vehicles:
            vehicles[v.id.data] = v.state
            if v.id.data not in recorded_ids:
                recorded_ids.append(v.id.data)
                screen_rect = screen.get_rect()
                all_sprites.add(Vehicle(screen_rect, v.id.data, (v.state.vec_position.x,
                                                                 v.state.vec_position.y, v.state.angle)))
        # 以自车位姿为视图中心；后续世界坐标投影均使用该局部参考系。
        center_3dof = (vehicles[ego_id].vec_position.x,
                       vehicles[ego_id].vec_position.y, vehicles[ego_id].angle)
        has_arena_info_dynamic = True

    def process_arena_info_static(self, data):
        """缓存自车附近的车道折线并转换为屏幕坐标。

        Args:
            data: ``ArenaInfoStatic`` 消息，包含不随仿真步变化的车道网络。
        """
        global lane_pts
        if has_arena_info_dynamic:
            visible_range = 150.0
            del lane_pts[:]
            for lane in data.lane_net.lanes:
                points = []
                for pt in lane.points:
                    if abs(pt.x - center_3dof[0]) < visible_range and abs(pt.y - center_3dof[1]) < visible_range:
                        points.append(project_world_to_image((pt.x, pt.y, 0.0)))
                if len(points) > 2:
                    lane_pts.append(points)

    def process_control_signal(self, data):
        """保存最近十个规划状态，用于显示速度、加速度和平均转角。

        Args:
            data: 自车 ``ControlSignal``，其中 ``state`` 是规划/控制目标状态。
        """
        global state_seq
        state_seq.append(data.state)
        if len(state_seq) > 10:
            state_seq.pop(0)

def project_world_to_image(point_3dof):
    """将世界坐标点投影到以自车为中心的 Pygame 像素坐标系。

    Args:
        point_3dof: ``(x, y, heading)`` 形式的世界坐标；当前只使用 x、y。

    Returns:
        tuple[float, float]: 画布中的 ``(u, v)`` 像素坐标。
    """
    x = point_3dof[0] - center_3dof[0]
    y = point_3dof[1] - center_3dof[1]
    angle = center_3dof[2]
    u = width / 2 + scale * (x * sin(angle) - y * cos(angle))
    v = height / 2 - scale * (x * cos(angle) + y * sin(angle))
    return (u, v)

class Wheel(pg.sprite.Sprite):
    """显示规划轨迹平均转角的方向盘精灵。"""

    def __init__(self, screen_rect):
        """加载方向盘图像并设置固定显示位置。

        Args:
            screen_rect: Pygame 画布矩形；为兼容精灵构造接口而保留。
        """
        pg.sprite.Sprite.__init__(self)
        self.original_image = pg.image.load("steer_wheel.png").convert_alpha()
        self.original_image = pg.transform.rotozoom(self.original_image, 0.0, 0.2)
        self.image = self.original_image
        self.rect = self.image.get_rect()
        self.rect.center = (100, 100)

    def update(self):
        """根据最近控制状态旋转方向盘图像，并保持图像中心不变。"""
        angle, acc = calc_current_steer_acc()
        angle = angle * 180 / pi
        self.image = pg.transform.rotate(self.original_image, angle)
        x, y = self.rect.center
        self.rect = self.image.get_rect()  # Replace old rect with new rect.
        self.rect.center = (x, y)  # Put the new rect's center at old center.

class Vehicle(pg.sprite.Sprite):
    """表示一辆仿真车辆的二维圆形精灵。"""

    def __init__(self, screen_rect, id, state_3dof):
        """创建车辆精灵并按自车/周车身份设置颜色。

        Args:
            screen_rect: Pygame 画布边界，保留供后续裁剪扩展使用。
            id: 仿真车辆唯一编号。
            state_3dof: 初始 ``(x, y, heading)`` 状态。
        """
        pg.sprite.Sprite.__init__(self)
        self.id = id
        self.state_3dof = state_3dof
        self.radius = 10
        self.image = pg.Surface((self.radius * 2, self.radius * 2), pg.SRCALPHA)
        if id == ego_id:
            pg.draw.circle(
                self.image, pg.Color('darkolivegreen1'), (self.radius, self.radius), self.radius)
        else:
            pg.draw.circle(
                self.image, pg.Color('dodgerblue1'), (self.radius, self.radius), self.radius)

        self.rect = self.image.get_rect(center=project_world_to_image(state_3dof))
        self.screen_rect = screen_rect

    def update(self):
        """读取该车辆最新状态并更新其屏幕位置。"""
        latest_state_3dof = (vehicles[self.id].vec_position.x,
                             vehicles[self.id].vec_position.y, vehicles[self.id].angle)
        self.rect.center = project_world_to_image(latest_state_3dof)

def calc_current_steer_acc():
    """由最近规划状态估计显示用转角，并返回最新加速度。

    Returns:
        tuple[float, float]: 放大后的方向盘显示角和最新纵向加速度；历史不足时
        返回 ``(0.0, 0.0)``。
    """
    if len(state_seq) < 2:
        return 0.0, 0.0
    steer_list = []
    for i in range(len(state_seq) - 1):
        state1 = state_seq[i]
        steer = atan(state1.curvature * 2.85)
        steer_list.append(steer)
    ave_steer = reduce(lambda x, y: x + y, steer_list) / len(steer_list)
    rotated_angle = ave_steer * 1.25 * (360.0 / 45.0)
    return rotated_angle, state_seq[-1].acceleration

def plot_lanes_on_screen():
    """绘制已投影到屏幕坐标的可见车道折线。"""
    lanepts_plot = list(lane_pts)
    for i in range(len(lanepts_plot)):
        pg.draw.lines(screen, pg.Color('deeppink'), False, lanepts_plot[i])

def plot_speed_on_screen():
    """在固定区域显示自车规划速度和加速度。"""
    speed = 0.0
    acc = 0.0
    if len(state_seq) > 1:
        speed = state_seq[-1].velocity * 3.6
        acc = state_seq[-1].acceleration
    font_obj = pg.font.Font('freesansbold.ttf', 20)
    text_surface_obj = font_obj.render(
        'vel: {:.2f} km/h '.format(speed), True, (0, 0, 0))
    text_rect_obj = text_surface_obj.get_rect()
    text_rect_obj.center = (110, 180)
    screen.blit(text_surface_obj, text_rect_obj)
    text_surface_obj = font_obj.render(
        'acc: {:.2f} m/s^2'.format(acc), True, (0, 0, 0))
    text_rect_obj = text_surface_obj.get_rect()
    text_rect_obj.center = (110, 200)
    screen.blit(text_surface_obj, text_rect_obj)

def plot_ids_on_screen():
    """在每辆已记录车辆附近绘制其仿真编号。"""
    for idx in recorded_ids:
        state_3dof = (vehicles[idx].vec_position.x,
                      vehicles[idx].vec_position.y, vehicles[idx].angle)
        font_obj = pg.font.Font('freesansbold.ttf', 16)
        text_surface_obj = font_obj.render(
            '{}'.format(idx), True, (0, 255, 0))
        text_rect_obj = text_surface_obj.get_rect()
        u, v = project_world_to_image(state_3dof)
        text_rect_obj.center = (u - 20, v)
        screen.blit(text_surface_obj, text_rect_obj)

def plot_orientations_on_screen():
    """绘制车辆相对自车朝向的短线段。"""
    for idx in recorded_ids:
        state_3dof = (vehicles[idx].vec_position.x,
                      vehicles[idx].vec_position.y, vehicles[idx].angle)
        u, v = project_world_to_image(state_3dof)
        angle_diff = vehicles[idx].angle - center_3dof[2]
        pt1 = (u, v)
        pt2 = (u - 10 * sin(angle_diff), v - 10 * cos(angle_diff))
        pg.draw.line(screen, pg.Color('black'), pt1, pt2, 3)

def plot_selected_rect_on_screen():
    """用矩形框标记当前接收键盘 HMI 指令的车辆。"""
    if agent_id in recorded_ids:
        agent_state = (vehicles[agent_id].vec_position.x,
                       vehicles[agent_id].vec_position.y, vehicles[agent_id].angle)
        u, v = project_world_to_image(agent_state)
        pg.draw.rect(screen, pg.Color('aquamarine3'), (u - 10, v - 10, 20, 20), 3)

def update_visualization():
    """在获得动态地图后按固定图层顺序重绘完整画面。"""
    if has_arena_info_dynamic:
        all_sprites.update()
        screen.fill(pg.Color('cornsilk2'))
        all_sprites.draw(screen)
        plot_lanes_on_screen()
        plot_ids_on_screen()
        plot_selected_rect_on_screen()
        plot_speed_on_screen()
        plot_orientations_on_screen()

def init_joy(frame_id):
    """创建满足当前 HMI 按键映射约定的零初始化 Joy 消息。

    Args:
        frame_id: 使用车辆编号编码的目标代理标识。

    Returns:
        Joy: 含 8 个轴和 11 个按钮的 ROS2 手柄消息。
    """
    joy = Joy()
    joy.header.frame_id = frame_id
    joy.header.stamp = Clock().now().to_msg()
    for i in range(8):
        joy.axes.append(0.0)
    for i in range(11):
        joy.buttons.append(0)
    return joy

def handle_keyboard_event(joy_pub):
    """处理鼠标选车和键盘行为指令，并发布对应 Joy 消息。

    Args:
        joy_pub: ``rclpy`` Joy 发布器。W/S/A/D 对应加速、制动和左右换道，
        Q/E/R 对应可行性状态及自动模式切换。
    """
    global agent_id
    for event in pg.event.get():
        if event.type == pg.MOUSEBUTTONUP:
            pos = pg.mouse.get_pos()
            clicked_sprites = [s for s in all_sprites if s.rect.collidepoint(pos)]
            if len(clicked_sprites) > 0:
                if hasattr(clicked_sprites[0], 'id'):
                    agent_id = clicked_sprites[0].id
                    print('update agent id to ', agent_id)

        if event.type == KEYDOWN:
            joy = init_joy("{}".format(agent_id))
            if event.key == pg.K_w:
                msg = 'Agent {}: Speed up'.format(agent_id)
                print(msg)
                joy.buttons[3] = 1
                joy_pub.publish(joy)
            elif event.key == pg.K_s:
                msg = 'Agent {}: Brake'.format(agent_id)
                print(msg)
                joy.buttons[0] = 1
                joy_pub.publish(joy)
            elif event.key == pg.K_a:
                msg = 'Agent {}: Lane change left'.format(agent_id)
                print(msg)
                joy.buttons[2] = 1
                joy_pub.publish(joy)
            elif event.key == pg.K_d:
                msg = 'Agent {}: Lane change right'.format(agent_id)
                print(msg)
                joy.buttons[1] = 1
                joy_pub.publish(joy)
            elif event.key == pg.K_q:
                msg = 'Agent {}: Toggle left lc feasible state'.format(agent_id)
                print(msg)
                joy.buttons[4] = 1
                joy_pub.publish(joy)
            elif event.key == pg.K_e:
                msg = 'Agent {}: Toggle right lc feasible state'.format(agent_id)
                print(msg)
                joy.buttons[5] = 1
                joy_pub.publish(joy)
            elif event.key == pg.K_r:
                msg = 'Agent {}: Toggle autonomous mode'.format(agent_id)
                print(msg)
                joy.buttons[6] = 1
                joy_pub.publish(joy)

def main(args=None):
    """初始化 Pygame 与 ROS2 节点，并进入 ROS2 事件循环。

    Args:
        args: 可选 ROS2 命令行参数。
    """
    rclpy.init(args=args)
    # Pygame 资源必须先于精灵创建完成初始化。
    pg.init()
    screen.fill(pg.Color('cornsilk3'))
    screen_rect = screen.get_rect()
    all_sprites.add(Wheel(screen_rect))
    all_sprites.draw(screen)
    pg.display.set_caption('Ultimate Vehicle Planning')
    pg.display.update()

    # 节点定时器负责刷新窗口，因此主线程只需交给 rclpy.spin 管理。
    node = TerminalServerNode()

    print('Terminal server initialized.')
    rclpy.spin(node)

    # ROS2 结束后按依赖顺序释放图形资源和通信上下文。
    pg.quit()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
