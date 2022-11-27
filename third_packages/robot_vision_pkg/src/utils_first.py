#!/usr/bin/env python3
# !coding=utf-8

import numpy as np
import math
import transforms3d as tfs
import params as pa

#from srv import CalRobot6D

def transT_forT(object_6D):
    x,y,z,rx,ry,rz = pa.T
    T_mat = eular2matrix_deg(x,y,z,rx,ry,rz)
    object_6D =object_6D.tolist()
    X,Y,Z= object_6D
    RX,RY,RZ = pa.table_rx,pa.table_ry,pa.table_rz
    post_mat = eular2matrix_deg(X,Y,Z,RX,RY,RZ)
    Robot_post_mat = np.dot(T_mat,post_mat)
    Robot_post =matrix2eular_deg(Robot_post_mat)
    return Robot_post[0],Robot_post[1],Robot_post[2]
    #return Robot_post
# Transformation between matrix and coordinates
def eular2matrix_deg(x, y, z, rx, ry, rz):
    rmat = tfs.euler.euler2mat(math.radians(rx), math.radians(ry), math.radians(rz))
    rmat = tfs.affines.compose(np.squeeze(np.asarray((x, y, z))), rmat, [1, 1, 1])
    return rmat


def eular2matrix_rad(x, y, z, rx, ry, rz):
    rmat = tfs.euler.euler2mat(rx, ry, rz)
    rmat = tfs.affines.compose(np.squeeze(np.asarray((x, y, z))), rmat, [1, 1, 1])
    return rmat


def matrix2eular_deg(m):
    rx, ry, rz = tfs.euler.mat2euler(m[0:3, 0:3])
    pos = np.squeeze(m[0:3, 3:4])
    return np.array([pos[0], pos[1], pos[2], math.degrees(rx), math.degrees(ry), math.degrees(rz)])


def matrix2eular_rad(m):
    rx, ry, rz = tfs.euler.mat2euler(m[0:3, 0:3])
    pos = np.squeeze(m[0:3, 3:4])
    return np.array([pos[0], pos[1], pos[2], rx, ry, rz])


def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def rot2quatMminimal(m):
    quat = tfs.quaternions.mat2quat(m[0:3, 0:3])
    return quat[1:]


def quatMinimal2rot(q):
    p = np.dot(q.T, q)
    w = np.sqrt(np.subtract(1, p[0][0]))
    return tfs.quaternions.quat2mat([w, q[0], q[1], q[2]])


# ---- Transformation between Camera-Matrix and Camera-Vector ----
def cameraVector2cameraMatrix(vec):  # [fx, fy, ppx, ppy]
    mat = np.float32([[vec[0], 0, vec[2]],
                      [0, vec[1], vec[3]],
                      [0, 0, 1]])
    return mat


def cameraMatrix2cameraVector(mat):
    vec = [mat[0, 0], mat[1, 1], mat[0, 2], mat[1, 2]]
    return vec


# ---- Transformation between Rotation-Matrix and Euler-Angles ----
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.type)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotationMatrix2eulerAngles(R):  # unit: rad
    assert (isRotationMatrix(R))
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])


def eulerAngles2rotationMatrix(r):
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(r[0]), -math.sin(r[0])],
                   [0, math.sin(r[0]), math.cos(r[0])]])
    Ry = np.array([[math.cos(r[1]), 0, math.sin(r[1])],
                   [0, 1, 0],
                   [-math.sin(r[1]), 0, math.cos(r[1])]])
    Rz = np.array([[math.cos(r[2]), -math.sin(r[2]), 0],
                   [math.sin(r[2]), math.cos(r[2]), 0],
                   [0, 0, 1]])
    R = np.dot(Rz, np.dot(Ry, Rx))
    return R


