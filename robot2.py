import matplotlib.pyplot as plt
import random
import math
from enum import Enum
import numpy as np
from matplotlib.patches import Ellipse

TIMESCALE = 0.1
TIME_BREAK = 70
NUM_ROBOT = 100
LOG_INTERVAL = 10


class Pos:
    x: float
    y: float

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y


class Vector2d:
    value: np.ndarray

    def __init__(self, value: np.ndarray):
        self.value = value

    def calc_eValue(self):
        v1 = np.array([0.5, 0.5])
        for _i in range(10):
            v1 = np.dot(self.value, v1)
            v1 = v1/np.linalg.norm(v1, ord=2)

        ev1 = np.linalg.norm(np.dot(self.value, v1), ord=2)
        v2 = np.dot(np.array([[math.cos(math.pi/2), -1*math.sin(math.pi/2)],
                              [math.sin(math.pi/2), math.cos(math.pi/2)]]), v1)
        ev2 = np.linalg.norm(np.dot(self.value, v2), ord=2)
        return [ev1, ev2], v1


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


class PositionListAtTime:
    time: int
    pos: np.ndarray
    avg: Pos
    eVector: Pos
    eValue: list
    v_cv_matrix: Vector2d

    def __init__(self, time: int, pos: np.ndarray):
        self.time = time
        self.pos = pos[:, :, time]
        self.calc_matrix()

    def calc_matrix(self):
        self.avg = Pos(np.average(self.pos[:, 0]), np.average(self.pos[:, 1]))
        print(self.pos.shape[1])
        _Sxx = 1/self.pos.shape[0] * np.dot((self.pos[:, 0] - self.avg.x), (self.pos[:, 0] - self.avg.x).T)
        _Syy = 1/self.pos.shape[0] * np.dot((self.pos[:, 1] - self.avg.y), (self.pos[:, 1] - self.avg.y).T)
        _Sxy = _Syx = 1/self.pos.shape[0] * np.dot((self.pos[:, 0] - self.avg.x), (self.pos[:, 1] - self.avg.y).T)
        self.v_cv_matrix = Vector2d([[_Sxx, _Sxy], [_Syx, _Syy]])
        print([[_Sxx, _Sxy], [_Syx, _Syy]])
        self.eValue, vector = self.v_cv_matrix.calc_eValue()
        self.eVector = Pos(*vector)
        #print(self.time, self.avg.x, self.avg.y, self.eVector.x, self.eVector.y, self.eValue, self.v_cv_matrix.value)


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
    def get_state(self):
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
        if False:
            robot_state = robot.get_state()
            print("x:{:.3f} y:{:.3f}, angle:{:.3f}".format(robot_state["position"].x,
                                                           robot_state["position"].y, robot_state["angle"]))

        result_x.append(robot.get_state()["position"].x)
        result_y.append(robot.get_state()["position"].y)

        ct += 1

    return (result_x, result_y)


def main() -> None:
    param = Param(0, 0, 0, 0, 0, 0)
    circle = run(param, setting_vel=1.0, setting_omg=1.0, time_break=TIME_BREAK)

    param = Param(0.01, 0.001, 0.001, 0.01, 0.001, 0.001)
    result = np.array([run(param, setting_vel=1.0, setting_omg=1.0, time_break=TIME_BREAK) for i in range(NUM_ROBOT)])

    fig = plt.figure(figsize=(6.0, 6.0))
    ax = fig.add_subplot(111)
    print(result.shape)
    # print(np.split(result,NUM_ROBOT,0)[0])
    group_list = []
    for i in range(0, TIME_BREAK, LOG_INTERVAL):
        group = PositionListAtTime(i, result)
        group_list.append(group.pos)
        #print(group.eVector.x, group.eVector.y)
        ell = Ellipse((group.avg.x, group.avg.y), math.sqrt(9.21*group.eValue[0]), math.sqrt(9.21*group.eValue[1]),
                      angle=math.atan2(group.eVector.y, group.eVector.x)*180/math.pi, alpha=0.5)
        ax.add_artist(ell)

    group_list = np.array(group_list)

    ax.scatter(group_list[:, :, 0], group_list[:, :, 1], s=2)
    ax.plot(circle[0], circle[1], color="b")
    # fig.show()

    fig.savefig("test.png")


if __name__ == '__main__':
    main()

# a = sqrt(9.21*λ1)
# b = sqrt(9.21*λ2)
# 99%の点群が入る値

#    # 各共分散行列ごとに固有ベクトルと固有値を求める
#    for i in range(0, len(sx2)):
#        v = (0.5, 0.5) # 初期ベクトル
#        for k in range(0, 10):
#            vx = sx2[i] * v[0] + sxy[i] * v[1]
#            vy = sxy[i] * v[0] + sy2[i] * v[1]
#            norm = math.sqrt(vx**2 + vy**2)
#            vx /= norm
#            vy /= norm
#            v = (vx, vy)
#        # 固有値(大きい方)
#        evx = sx2[i] * v[0] + sxy[i] * v[1]
#        evy = sxy[i] * v[0] + sy2[i] * v[1]
#        ev1 = math.sqrt(evx**2 + evy**2)
#
#        # 固有値（小さい方）
#        v2x = math.cos(math.pi/2) * v[0] - math.sin(math.pi/2) * v[1]
#        v2y = math.sin(math.pi/2) * v[0] + math.cos(math.pi/2) * v[1]
#        ev2x = sx2[i] * v2x + sxy[i] * v2y
#        ev2y = sxy[i] * v2x + sy2[i] * v2y
#        ev2 = math.sqrt(ev2x**2 + ev2y**2)
#        # 誤差楕円の描画
#        ell = Ellipse((average_x[i], average_y[i]), math.sqrt(9.2*ev1), math.sqrt(9.2*ev2),
#                angle=math.atan2(v[1], v[0])*180/math.pi, alpha=0.5)
#        ax.add_artist(ell)