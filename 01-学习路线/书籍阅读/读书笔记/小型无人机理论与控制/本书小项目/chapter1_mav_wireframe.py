import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 模型尺寸参数（单位自定，比例即可）
PARAMS = {
    "fuse_l1": 1.6,   # 机头到机身基准面距离
    "fuse_l3": 7.0,   # 机身总长度（到尾锥尖）
    "fuse_w": 1.2,    # 机身基准面宽度
    "fuse_h": 0.9,    # 机身基准面高度
    "wing_w": 7.5,    # 机翼翼展
    "wing_l": 1.6,    # 机翼前后长度
    "tail_w": 3.0,    # 水平尾翼翼展
    "tail_l": 1.1,    # 水平尾翼前后长度
    "tail_h": 1.6,    # 垂尾高度
}

# 动画开关：False 时可用鼠标拖拽查看模型
ANIMATE = False


def build_vertices(p):
    # 机身：两个矩形底锥体共用同一矩形底（书中三视图结构）
    nose = np.array([0.0, 0.0, 0.0])   # 机头锥尖
    x1 = p["fuse_l1"]                  # 共享矩形底所在平面
    x3 = p["fuse_l3"]                  # 尾部锥尖

    w = p["fuse_w"]
    h = p["fuse_h"]

    b1 = np.array([x1, -w / 2, -h / 2])
    b2 = np.array([x1,  w / 2, -h / 2])
    b3 = np.array([x1,  w / 2,  h / 2])
    b4 = np.array([x1, -w / 2,  h / 2])

    tail = np.array([x3, 0.0, 0.0])    # 尾锥尖

    # 机翼（矩形穿过机身）
    wing_x = x1 + (p["fuse_l3"] - x1) * 0.15
    w1p = np.array([wing_x, -p["wing_w"] / 2, 0.0])
    w2p = np.array([wing_x,  p["wing_w"] / 2, 0.0])
    w3p = np.array([wing_x + p["wing_l"],  p["wing_w"] / 2, 0.0])
    w4p = np.array([wing_x + p["wing_l"], -p["wing_w"] / 2, 0.0])

    # 水平尾翼
    tail_x = x3 - p["tail_l"]
    t1 = np.array([tail_x, -p["tail_w"] / 2, 0.0])
    t2 = np.array([tail_x,  p["tail_w"] / 2, 0.0])
    t3 = np.array([tail_x + p["tail_l"],  p["tail_w"] / 2, 0.0])
    t4 = np.array([tail_x + p["tail_l"], -p["tail_w"] / 2, 0.0])

    # 垂尾（XOZ 平面直角三角形）
    v_base1 = np.array([tail_x + p["tail_l"] * 0.4, 0.0, 0.0])
    v_base2 = np.array([tail_x + p["tail_l"], 0.0, 0.0])
    v_tip = np.array([tail_x + p["tail_l"], 0.0, p["tail_h"]])

    return {
        "nose": nose,
        "base": [b1, b2, b3, b4],
        "tail": tail,
        "wing": [w1p, w2p, w3p, w4p],
        "htail": [t1, t2, t3, t4],
        "vtail": [v_base1, v_base2, v_tip],
    }


def rot_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    r_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    r_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    r_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    return r_z @ r_y @ r_x


def apply_transform(points, roll, pitch, yaw, translation):
    r = rot_matrix(roll, pitch, yaw)
    return [r @ p + translation for p in points]


def draw_polyline(ax, pts, close=True, **kwargs):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    if close:
        xs.append(pts[0][0])
        ys.append(pts[0][1])
        zs.append(pts[0][2])
    ax.plot(xs, ys, zs, **kwargs)


def draw_pyramid(ax, apex, base_pts, **kwargs):
    # 画底面
    draw_polyline(ax, base_pts, close=True, **kwargs)
    # 画四条侧边
    for bp in base_pts:
        ax.plot(
            [apex[0], bp[0]],
            [apex[1], bp[1]],
            [apex[2], bp[2]],
            **kwargs,
        )


def build_lines(ax, v, roll, pitch, yaw, translation):
    # 机身线框（两个共底四棱锥）
    base_t = apply_transform(v["base"], roll, pitch, yaw, translation)
    nose_t = apply_transform([v["nose"]], roll, pitch, yaw, translation)[0]
    tail_t = apply_transform([v["tail"]], roll, pitch, yaw, translation)[0]

    draw_pyramid(ax, nose_t, base_t, color="#1f77b4", linewidth=1.2)
    draw_pyramid(ax, tail_t, base_t, color="#1f77b4", linewidth=1.2)

    # 机翼/尾翼/垂尾
    for pts in [v["wing"], v["htail"], v["vtail"]]:
        t_pts = apply_transform(pts, roll, pitch, yaw, translation)
        draw_polyline(ax, t_pts, close=True, color="#1f77b4", linewidth=1.2)


def main():
    v = build_vertices(PARAMS)

    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("MAV Wireframe (Python Minimal)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # 坐标轴范围
    lim = PARAMS["fuse_l3"] + 4
    ax.set_xlim(0, lim)
    ax.set_ylim(-lim / 2, lim / 2)
    ax.set_zlim(-lim / 4, lim / 4)

    def update(frame):
        ax.cla()
        ax.set_title("MAV Wireframe (Python Minimal)")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_xlim(0, lim)
        ax.set_ylim(-lim / 2, lim / 2)
        ax.set_zlim(-lim / 4, lim / 4)

        t = frame / 50.0
        roll = 0.2 * math.sin(t)
        pitch = 0.15 * math.cos(t * 0.7)
        yaw = 0.2 * t
        translation = np.array([t * 0.5, 0.0, 0.0])

        build_lines(ax, v, roll, pitch, yaw, translation)

    if ANIMATE:
        FuncAnimation(fig, update, frames=300, interval=30)
    else:
        build_lines(ax, v, 0.0, 0.0, 0.0, np.array([0.0, 0.0, 0.0]))

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
