import matplotlib.pyplot as plt
import random
import math
from enum import Enum

TIMESCALE = 0.01
TIME_BREAK = 3000


class Pos:
    x: float
    y: float

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y


class Param:
    a1: float
    a2: float
    a3: float
    a4: float
    a5: float
    a6: float

    def __init__(self, a1: float, a2: float, a3: float, a4: float, a5: float, a6: float):
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.a4 = a4
        self.a5 = a5
        self.a6 = a6


class Command:
    class Mode(Enum):
        GO_FORWARD = 1
        GO_HOME = 2
        STAY = 3

    class UndefinedCommandError(Exception):
        pass
    velocity: float
    angular_velocity: float
    mode: Mode

    def __init__(self, v: float, a: float, mode: Mode):
        self.velocity = v
        self.angular_velocity = a
        self.mode = mode


# 指令発行者
class Commander:
    param: Param
    setting_vel: float
    # omega (angular velocity)
    setting_omg: float

    def __init__(self, param: Param, setting_vel: float, setting_omg: float):
        self.param = param
        self.setting_vel = setting_vel
        self.setting_omg = setting_omg

    # ロボットへ指令を送信
    def send_command(self, mode: Command.Mode):
        _v = self.setting_vel + \
            random.gauss(0,
                         math.sqrt((self.param.a1 * self.setting_vel ** 2) + (self.param.a2 * self.setting_omg ** 2)))
        _a = self.setting_omg + \
            random.gauss(0,
                         math.sqrt((self.param.a3 * self.setting_vel ** 2) + (self.param.a4 * self.setting_omg ** 2)))
        return Command(_v, _a, mode)


# ロボットクラス定義
class Robot:
    before_time: int
    pos: Pos
    home_pos: Pos
    angle: float

    def __init__(self, home_pos: Pos = Pos(0, 0), pos: Pos = None, angle: float = 0, before_time: int = 0):
        self.home_pos = home_pos
        if pos:
            self.pos = pos
        else:
            self.pos = home_pos
        self.angle = angle
        self.before_time = before_time

    # 移動後のランダム回転
    def random_rotate(self, cmd: Command, a5: float, a6: float):
        return random.gauss(0, math.sqrt((a5 * cmd.velocity ** 2) + (a6 * cmd.angular_velocity ** 2)))

    def receive_command(self, cmd: Command, time: int, param: Param) -> None:
        if cmd.mode == Command.Mode.GO_FORWARD:
            self.move_forward(cmd, time, param)
        elif cmd.mode == Command.Mode.GO_HOME:
            self.move_force_home()
        elif cmd.mode == Command.Mode.STAY:
            pass
        else:
            raise cmd.UndefinedCommandError("Unknown command")
        self.before_time = time

    # 指令にしたがって前進
    def move_forward(self, cmd: Command, time: int, param: Param) -> None:
        delta_time = (time - self.before_time) * TIMESCALE

        self.pos.x = self.pos.x - \
            cmd.velocity/cmd.angular_velocity * math.sin(self.angle) + \
            cmd.velocity/cmd.angular_velocity * math.sin(self.angle + (cmd.angular_velocity * delta_time))

        self.pos.y = self.pos.y + \
            cmd.velocity/cmd.angular_velocity * math.cos(self.angle) - \
            cmd.velocity/cmd.angular_velocity * math.cos(self.angle + (cmd.angular_velocity * delta_time))

        self.angle = self.angle + cmd.angular_velocity * delta_time + \
            self.random_rotate(cmd, param.a5, param.a6) * delta_time

    # 強制的にホームポジションへ移動
    def move_force_home(self) -> None:
        self.move_force(self.home_pos)

    # 強制的に移動
    def move_force(self, dst: Pos) -> None:
        self.pos = dst

    # 現在位置のログ取得
    def get_position(self):
        return {"position": self.pos, "angle": self.angle}


def run(param: Param, setting_vel: float, setting_omg: float, time_break: int):

    # ロボットの配置
    robot = Robot(home_pos=Pos(0, 0))
    # コントローラの作成
    cmd_generator = Commander(param, setting_vel, setting_omg)

    ct = 0
    result_x = []
    result_y = []

    while True:
        if ct >= time_break:
            break

        # ノイズが含まれたコマンド送信
        cmd = cmd_generator.send_command(mode=Command.Mode.GO_FORWARD)
        robot.receive_command(cmd, ct, param)
        # print(robot.get_position()["position"].x)

        result_x.append(robot.get_position()["position"].x)
        result_y.append(robot.get_position()["position"].y)

        ct += 1

    return (result_x, result_y)


def main() -> None:
    param = Param(0, 0, 0, 0, 0, 0)
    x1, y1 = run(param, setting_vel=1.0, setting_omg=1.0, time_break=TIME_BREAK)

    param = Param(0.01, 0.001, 0.001, 0.01, 0.001, 0.1)
    x2, y2 = run(param, setting_vel=1.0, setting_omg=1.0, time_break=TIME_BREAK)

    fig = plt.figure(figsize=(6.0, 6.0))
    ax = fig.add_subplot(111)
    ax.plot(x1, y1, color="r")
    ax.plot(x2, y2, color="b")

    fig.savefig("robot_04192.png")


if __name__ == '__main__':
    main()
