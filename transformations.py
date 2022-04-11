#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

def rpy2qua(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) *np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) *np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) *np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) *np.sin(yaw/2)
    return np.array([qx, qy, qz, qw])
    
def qua2rpy(quaternion):
    qx = quaternion[0]
    qy = quaternion[1]
    qz = quaternion[2]
    qw = quaternion[3]
    roll = math.atan2(2 * (qw * qx + qy * qz), (1 - 2 * (qx**2 + qy**2)))
    pitch = math.asin(2 * (qw * qy - qz * qx))
    yaw = math.atan2(2 * (qw * qz + qx * qy), (1 - 2 * (qy**2 + qz**2)))
    return np.array([roll, pitch, yaw])
    
def rotv2rotm(rotv, result = "homogeneo"):
    I = np.eye(3)
    p = rotv[0, 0:3]
    r = rotv[0, 3:6]
    theta = math.sqrt(r[0,0] ** 2+ r[0,1] ** 2+ r[0,2] ** 2)
    if theta <= 0:
        R=I
    else:
        a = r/theta
        skew_sym_mat = np.mat([[0, -a[0,2], a[0,1]], [a[0,2], 0, -a[0,0]], [-a[0,1], a[0,0], 0]])
        R = math.cos(theta) * I + (1- math.cos(theta)) * np.dot(np.transpose(a), a) + math.sin(theta) * skew_sym_mat
        J = math.sin(theta) / theta * I + (1 - math.sin(theta)/theta) * np.dot(np.transpose(a) ,a) + (1 - math.cos(theta)) / theta * skew_sym_mat
        P = np.dot(J, np.transpose(p))
    rotm = np.insert(R, 3, values = p, axis = 1)
    rotm = np.insert(rotm, 3, values = [0, 0, 0, 1], axis = 0)
    if result == "homogeneo":
        return rotm
    elif result == "rotation":
        return R
    else:
        print("Aviso: Revisa qué matriz necesitas, matriz de rotación ó matriz homogenea")
        
def rotm2rotv(rotm):
    diag = rotm[0,0]+rotm[1,1]+rotm[2,2]
    theta = math.acos((diag-1)/2)
    ry = (rotm[0,2] - rotm[2,0]) / 2 * theta / np.sin(theta)
    rz = (rotm[0,1] - rotm[1,0]) / 2 * theta / np.sin(theta) 
    rx = (rotm[2,1] - rotm[1,2]) / 2 * theta / np.sin(theta)
    return np.array([rx, ry, rz])

def rotm2rpy(rotm):
    pitch = math.asin(-rotm[2,0])
    roll = math.acos(rotm[2,2]/ math.cos(pitch))
    yaw = math.acos(rotm[0,0]/ math.cos(pitch))
    return np.array([roll, pitch, yaw])
    
def rotm2qua(rotm):
    rotm = rotm[0:3,0:3]
    trR = float(rotm.trace())
    w = math.sqrt(trR + 1) / 2
    x = (rotm[2, 1]-rotm[1, 2]) / (4 * w)
    y = (rotm[0, 2]-rotm[2, 0]) / (4 * w)
    z = (rotm[1, 0]-rotm[0, 1]) / (4 * w)
    return np.array([x, y, z, w])

def qua2rotm(quaternion):
    qx, qy, qz, qw = quaternion[0,0], quaternion[0,1], quaternion[0,2], quaternion[0,3]
    m11 = 1 - 2 * (qy ** 2) - 2 * (qz ** 2)
    m12 = 2 * qx * qy - 2 * qw * qz
    m13 = 2 * qx * qz + 2 * qw * qy
    m21 = 2 * qx * qy + 2 * qw * qz
    m22 = 1 - 2 * (qx ** 2) - 2 * (qz ** 2)
    m23 = 2 * qy * qz - 2 * qw * qx
    m31 = 2 * qx * qz - 2 * qw * qy
    m32 = 2 * qy * qz + 2 * qw * qx
    m33 = 1 - 2 * (qx ** 2) - 2 * (qy ** 2)
    return np.matrix([[m11, m12, m13], [m21, m22, m23], [m31, m32, m33]])

def rotv2qua(rotv):
    if rotv.size() > 3:
        theta = math.sqrt(rotv[0,3] ** 2+ rotv[0,4] ** 2+ rotv[0,5] ** 2)
        qw = math.cos(theta / 2)
        qx = rotv[0, 3] / theta * asin(theta / 2)
        qy = rotv[0, 4] / theta * asin(theta / 2)
        qz = rotv[0, 5] / theta * asin(theta / 2)
    else:
        theta = math.sqrt(rotv[0,0] ** 2+ rotv[0,1] ** 2+ rotv[0,2] ** 2)
        qw = math.cos(theta / 2)
        qx = rotv[0, 0] / theta * asin(theta / 2)
        qy = rotv[0, 1] / theta * asin(theta / 2)
        qz = rotv[0, 2] / theta * asin(theta / 2)
    return np.array([qx, qy, qz, qw])

def qua2rotv(qua):
    theta = 2 * acos(qua[0, 3])
    kx = qua[0, 0] / math.sqrt(1 - qua[0, 3] ** 2)
    ky = qua[0, 1] / math.sqrt(1 - qua[0, 3] ** 2)
    kz = qua[0, 2] / math.sqrt(1 - qua[0, 3] ** 2)
    return np.array([kx, ky, kz])
