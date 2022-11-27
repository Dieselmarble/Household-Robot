#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# yangshihua 2021/6/30

import cv2
import numpy as np
import sys
from params import axes, corners
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import transforms3d as tfs

np.set_printoptions(suppress=True)
np.set_printoptions(precision=9)



# ---- Undistort image----
def undistort(img, camera_matrix, dist_coefs):
    h, w = img.shape[:2]
    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coefs, None, camera_matrix, (w,h), 5)
    undist_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
    return undist_img

# ---- Visualize detected positions and orientations ----
def drawAxes(img, origin, axes_pixels, tid):
    origin_pix = tuple(origin.astype(int))
    xpts = tuple(axes_pixels[0].astype(int))
    ypts = tuple(axes_pixels[1].astype(int))
    zpts = tuple(axes_pixels[2].astype(int))

    img = cv2.line(img, origin_pix, xpts, (255, 0, 0), 5)
    img = cv2.line(img, origin_pix, ypts, (0, 255, 0), 5)
    img = cv2.line(img, origin_pix, zpts, (0, 0, 255), 5)

    # cv2.putText(img, '(%.3f, %.3f)' % (origin_pix[0], origin_pix[1]), tuple(np.array(origin_pix)+5), 1, 5, (255, 255, 0), 5, cv2.LINE_AA)
    cv2.putText(img, '%s' % tid, tuple(np.array(origin_pix) + 10), 1, 5, (255, 255, 0), 5, cv2.LINE_AA)
    cv2.putText(img, 'x', tuple(np.array(xpts)+5), 1, 5, (255, 0, 0), 5)
    cv2.putText(img, 'y', tuple(np.array(ypts)+5), 1, 5, (0, 255, 0), 5)
    cv2.putText(img, 'z', tuple(np.array(zpts)+5), 1, 5, (0, 0, 255), 5)
    return img

def drawCorners(img, corners):
    for i in range(len(corners)):
        corner = corners[i].astype(int)
        cv2.circle(img, tuple(corner), 3, (255, 0, 255), 2)
        cv2.putText(img, '%s' % (i+1), tuple(corner+5), 1, 5, (255, 0, 255), 5)
    return img

def visTagCoordinates(img, tags, camera_matrix, dist_coefs):
    for tag in tags:
        tid = tag.tag_id
        R = tag.pose_R
        rvec, _ = cv2.Rodrigues(R)
        tvec = tag.pose_t
        origin = tag.center

        axes_pixels, _ = cv2.projectPoints(axes, rvec, tvec, camera_matrix, dist_coefs)
        corners_pixels, _ = cv2.projectPoints(corners, rvec, tvec, camera_matrix, dist_coefs)
        img = drawAxes(img, origin, axes_pixels.squeeze(), tid)
        img = drawCorners(img, corners_pixels.squeeze())
    return img


def vis3Daxes():
    import numpy as np
    import matplotlib.pyplot as plt

    stike = np.linspace(50, 150, 24)
    ttm = np.linspace(0.5, 2.5, 24)
    stike, ttm = np.meshgrid(stike, ttm)
    print(stike[:2])

    iv = (stike - 100) ** 2 / (100 * stike) / ttm
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure(figsize=(9, 6))
    ax = fig.gca(projection='3d')
    surf = ax.plot_surface(stike, ttm, iv, rstride=2, cstride=2, cmap=plt.cm.coolwarm, linewidth=0.5, antialiased=True)
    ax.set_xlabel('strike')
    ax.set_ylabel('time-to-maturity')
    ax.set_zlabel('implied volatility')

    plt.show()


def visualize3D_samples():
    # 定义坐标轴
    fig = plt.figure()
    ax1 = plt.axes(projection='3d')
    # ax = fig.add_subplot(111,projection='3d')  #这种方法也可以画多个子图

    # 定义图像和三维格式坐标轴
    fig = plt.figure()
    ax2 = Axes3D(fig)

    z = np.linspace(0, 13, 1000)
    x = 5 * np.sin(z)
    y = 5 * np.cos(z)
    zd = 13 * np.random.random(100)
    xd = 5 * np.sin(zd)
    yd = 5 * np.cos(zd)
    ax1.scatter3D(xd, yd, zd, cmap='Blues')  # 绘制散点图
    ax2.plot3D(x, y, z, 'gray')  # 绘制空间曲线
    plt.show()

    fig = plt.figure()  # 定义新的三维坐标轴
    ax3 = plt.axes(projection='3d')

    # 定义三维数据
    xx = np.arange(-5, 5, 0.5)
    yy = np.arange(-5, 5, 0.5)
    X, Y = np.meshgrid(xx, yy)
    Z = np.sin(X) + np.cos(Y)

    # 作图
    ax3.plot_surface(X, Y, Z, cmap='rainbow')
    # ax3.contour(X,Y,Z, zdim='z',offset=-2，cmap='rainbow)   #等高线图，要设置offset，为Z的最小值
    ax3.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='rainbow')
    plt.show()

    # 定义坐标轴
    fig4 = plt.figure()
    ax4 = plt.axes(projection='3d')

    # 生成三维数据
    xx = np.arange(-5, 5, 0.1)
    yy = np.arange(-5, 5, 0.1)
    X, Y = np.meshgrid(xx, yy)
    Z = np.sin(np.sqrt(X ** 2 + Y ** 2))

    # 作图
    ax4.plot_surface(X, Y, Z, alpha=0.3, cmap='winter')  # 生成表面， alpha 用于控制透明度
    ax4.contour(X, Y, Z, zdir='z', offset=-3, cmap="rainbow")  # 生成z方向投影，投到x-y平面
    ax4.contour(X, Y, Z, zdir='x', offset=-6, cmap="rainbow")  # 生成x方向投影，投到y-z平面
    ax4.contour(X, Y, Z, zdir='y', offset=6, cmap="rainbow")  # 生成y方向投影，投到x-z平面
    # ax4.contourf(X,Y,Z,zdir='y', offset=6,cmap="rainbow")   #生成y方向投影填充，投到x-z平面，contourf()函数

    # 设定显示范围
    ax4.set_xlabel('X')
    ax4.set_xlim(-6, 4)  # 拉开坐标轴范围显示投影
    ax4.set_ylabel('Y')
    ax4.set_ylim(-4, 6)
    ax4.set_zlabel('Z')
    ax4.set_zlim(-3, 3)

    plt.show()

    # 定义坐标轴
    fig4 = plt.figure()
    ax4 = plt.axes(projection='3d')

    # 生成三维数据
    xx = np.random.random(20) * 10 - 5  # 取100个随机数，范围在5~5之间
    yy = np.random.random(20) * 10 - 5
    X, Y = np.meshgrid(xx, yy)
    Z = np.sin(np.sqrt(X ** 2 + Y ** 2))

    # 作图
    ax4.scatter(X, Y, Z, alpha=0.3, c=np.random.random(400),
                s=np.random.randint(10, 20, size=(20, 40)))  # 生成散点.利用c控制颜色序列,s控制大小

    # 设定显示范围

    plt.show()


# Visualize 6D pose
def draw6DReference(ax):
    r = 0.1
    axes = np.float32([[0, 0, 0],
                       [r, 0, 0],
                       [0, r, 0],
                       [0, 0, r]])  # 3D coordinates
    x_axis = np.row_stack((axes[0], axes[1]))
    y_axis = np.row_stack((axes[0], axes[2]))
    z_axis = np.row_stack((axes[0], axes[3]))
    ax.plot3D(x_axis[:, 0], x_axis[:, 1], x_axis[:, 2], 'blue')
    ax.plot3D(y_axis[:, 0], y_axis[:, 1], y_axis[:, 2], 'green')
    ax.plot3D(z_axis[:, 0], z_axis[:, 1], z_axis[:, 2], 'red')

def draw6DPose(pose_6D, ax):    # x,y,z,rx,ry,rz (mm & rad)
    pose_t = np.squeeze(pose_6D[:3])
    pose_R = np.squeeze(tfs.euler.euler2mat(pose_6D[3], pose_6D[4], pose_6D[5]))

    r = 0.05
    axes = np.float32([[0, 0, 0],
                       [r, 0, 0],
                       [0, r, 0],
                       [0, 0, r]])  # 3D coordinates
    for i in range(axes.shape[0]):
        axes[i] = np.dot(pose_R, axes[i])+pose_t
    print(axes)

    x_axis = np.row_stack((axes[0], axes[1]))
    y_axis = np.row_stack((axes[0], axes[2]))
    z_axis = np.row_stack((axes[0], axes[3]))
    ax.plot3D(x_axis[:, 0], x_axis[:, 1], x_axis[:, 2], 'blue')
    ax.plot3D(y_axis[:, 0], y_axis[:, 1], y_axis[:, 2], 'green')
    ax.plot3D(z_axis[:, 0], z_axis[:, 1], z_axis[:, 2], 'red')

    x_min, x_max = np.min(axes[:, 0]), np.max(axes[:, 0])
    y_min, y_max = np.min(axes[:, 1]), np.max(axes[:, 1])
    z_min, z_max = np.min(axes[:, 2]), np.max(axes[:, 2])

    return x_min, x_max, y_min, y_max, z_min, z_max


def visMarker6DPoses():
    camera = np.loadtxt('./coordinates_camera_1030/camera_coordinates.txt').tolist()

    fig = plt.figure()
    ax = Axes3D(fig)

    draw6DReference(ax)
    x_min_fin, x_max_fin, y_min_fin, y_max_fin, z_min_fin, z_max_fin = 0, 0, 0, 0, 0, 0
    for i in range(len(camera)):
        x_min, x_max, y_min, y_max, z_min, z_max = draw6DPose(camera[i], ax)
        if x_min < x_min_fin:
            x_min_fin = x_min
        if x_max > x_max_fin:
            x_max_fin = x_max
        if y_min < y_min_fin:
            y_min_fin = y_min
        if y_max > y_max_fin:
            y_max_fin = y_max
        if z_min < z_min_fin:
            z_min_fin = z_min
        if z_max > z_max_fin:
            z_max_fin = z_max

    min_all = np.min((x_min_fin, y_min_fin, z_min_fin))
    max_all = np.max((x_max_fin, y_max_fin, z_max_fin))
    dist = max_all - min_all

    ax.set_xlabel('X')
    ax.set_xlim(min_all-dist/2, max_all-dist/2)  # 拉开坐标轴范围显示投影
    ax.set_ylabel('Y')
    ax.set_ylim(min_all-dist/2, max_all-dist/2)
    ax.set_zlabel('Z')
    ax.set_zlim(min_all, max_all)

    plt.show()
    plt.close()



if __name__ == '__main__':
    # vis3Daxes()
    # visualize3D()

    visMarker6DPoses()
