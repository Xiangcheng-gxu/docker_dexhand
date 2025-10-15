import torch
import numpy as np
import matplotlib.pyplot as plt
import json
from manotorch.manolayer import ManoLayer, MANOOutput
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection
import math
import time
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
import open3d as o3d
import os
from typing import Any, Dict

CONNECTIONS = [
    [0, 1], [0, 5], [0, 9], [0, 13], [0, 17],# 手腕到各手指根部
    [1, 2], [2, 3], [3, 4],# 拇指
    [5, 6], [6, 7], [7, 8],# 食指
    [9, 10], [10, 11], [11, 12],# 中指
    [13, 14], [14, 15], [15, 16],# 无名指
    [17, 18], [18, 19], [19, 20]# 小指
]

connections = [
    # 拇指
    [0, 1], [1, 2], [2, 3], [3, 4],
    # 食指
    [0, 5], [5, 6], [6, 7], [7, 8],
    # 中指
    [0, 9], [9, 10], [10, 11], [11, 12],
    # 无名指
    [0, 13], [13, 14], [14, 15], [15, 16],
    # 小指
    [0, 17], [17, 18], [18, 19], [19, 20]
]

change_bone_connections = [
    [[1, 2], [5, 6], [9, 10], [13, 14], [17, 18]],
    [[2, 3], [6, 7], [10, 11], [14, 15], [18, 19]],
    [[3, 4], [7, 8], [11, 12], [15, 16], [19, 20]],
]

change_mano_pose_connections = [
    [[5, 6], [9, 10], [17, 18], [13, 14], [1, 2]],
    [[6, 7], [10, 11], [18, 19], [14, 15], [2, 3]],
    [[7, 8], [11, 12], [19, 20], [15, 16], [3, 4]],
]

FINGER_BONES = [
    [1, 2], [2, 3], [3, 4],# 拇指
    [5, 6], [6, 7], [7, 8],# 食指
    [9, 10], [10, 11], [11, 12],# 中指
    [13, 14], [14, 15], [15, 16],# 无名指
    [17, 18], [18, 19], [19, 20]# 小指
]

# def project_to_xOz(vector):
#     """
#     将三维向量投影到 XOZ 平面 (y=0 平面)
#     :param vector: 输入的三维向量 [x, y, z]
#     :return: 投影后的向量 [x, 0, z]
#     """
#     return np.array([vector[0], 0, vector[2]])

# def project_to_xOy(vector):
#     """
#     将三维向量投影到 XOY 平面 (z=0 平面)
#     :param vector: 输入的三维向量 [x, y, z]
#     :return: 投影向量 [x, y, 0]
#     """
#     return np.array([vector[0], vector[1], 0])

# def project_to_yOz(vector):
#     """
#     将三维向量投影到 XOY 平面 (z=0 平面)
#     :param vector: 输入的三维向量 [x, y, z]
#     :return: 投影向量 [x, y, 0]
#     """
#     return np.array([vector[0], vector[1], 0])


def rotate_point_cloud(pc1, R):
    assert R.shape == (3, 3)
    print(pc1.shape)
    print(R.shape)
    pc0 = np.dot(pc1, R) 
    return pc0

def rotate_point_cloud_inv(pc1, R):
    assert R.shape == (3, 3)
    R_inv = R.T
    pc0 = np.dot(pc1, R_inv) 
    return pc0

##########################################################################################################
def vector_projections_and_rotation(vector):
    """
    计算一个三维向量在xoy, yoz, xoz坐标平面上的投影，并计算从投影向量旋转回原向量的旋转矩阵。
    
    参数:
    vector -- 一个三维数组或列表，表示空间中的向量[x, y, z]
    
    返回:
    projections -- 字典，包含在xoy, yoz, xoz平面上的投影向量
    rotation_matrices -- 字典，包含从各投影向量旋转回原向量的旋转矩阵
    """
    # 将输入转换为NumPy数组以确保后续计算
    v_original = np.array(vector, dtype=float)
    
    # 1. 计算向量在三个坐标平面上的投影
    # 投影原理：将垂直于目标平面的分量置零
    # XOY平面 (Z=0): 忽略Z分量
    proj_xoy = np.array([v_original[0], v_original[1], 0.0])
    # YOZ平面 (X=0): 忽略X分量
    proj_yoz = np.array([0.0, v_original[1], v_original[2]])
    # XOZ平面 (Y=0): 忽略Y分量
    proj_xoz = np.array([v_original[0], 0.0, v_original[2]])
    
    projections = {
        'xoy': proj_xoy,
        'yoz': proj_yoz,
        'xoz': proj_xoz
    }
    
    # 2. 计算从投影向量旋转回原向量的旋转矩阵
    rotation_matrices = {}
    # 对于每个投影，旋转轴是投影向量与原向量的叉积（方向垂直），旋转角是两向量间的夹角
    for plane, proj in projections.items():
        # 检查投影向量是否为零向量（避免除以零）
        if np.linalg.norm(proj) < 1e-10 or np.linalg.norm(v_original) < 1e-10:
            # 如果投影或原向量是零向量，则无法定义旋转矩阵，返回单位矩阵
            rotation_matrices[plane] = np.eye(3)
            continue
            
        # 计算旋转轴：投影向量与原向量的叉积，得到垂直于两者所在平面的轴
        rotation_axis = np.cross(proj, v_original)
        # 如果叉积结果很接近零向量（即原向量与投影向量几乎平行），则使用单位矩阵
        if np.linalg.norm(rotation_axis) < 1e-10:
            rotation_matrices[plane] = np.eye(3)
            continue
            
        # 单位化旋转轴
        rotation_axis_unit = rotation_axis / np.linalg.norm(rotation_axis)
        
        # 计算旋转角度：原向量与投影向量之间的夹角
        # 因为投影向量的模长小于或等于原向量，点积除以模长乘积可以得到夹角余弦
        cos_theta = np.dot(proj, v_original) / (np.linalg.norm(proj) * np.linalg.norm(v_original))
        # 由于浮点数精度问题，确保cos_theta在[-1, 1]范围内
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        theta = np.arccos(cos_theta)
        
        # 使用Scipy的Rotation.from_rotvec根据旋转轴和旋转角创建旋转对象
        rot_vec = rotation_axis_unit * theta
        rotation_obj = R.from_rotvec(rot_vec)
        # 获取3x3旋转矩阵
        rotation_matrix = rotation_obj.as_matrix()
        
        rotation_matrices[plane] = rotation_matrix
    
    return projections, rotation_matrices

def calculate_angles_with_negative_x_axis(v):
    """
    计算向量在xoy平面和xoz平面上与x轴负方向的夹角（单位：度）。
    
    参数:
    v -- 输入向量，形式为 [x, y, z] 或 (x, y, z)
    
    返回:
    angle_xy -- 在xoy平面上的夹角（度）
    angle_xz -- 在xoz平面上的夹角（度）
    """
    x, y, z = v
    # 计算在xoy平面上的夹角
    denom_xy = np.sqrt(x**2 + y**2)
    if denom_xy == 0:
        angle_xy = np.nan  # 未定义
    else:
        cos_theta_xy = -x / denom_xy
        # 防止浮点误差导致arccos超出[-1,1]
        cos_theta_xy = np.clip(cos_theta_xy, -1.0, 1.0)
        theta_xy_rad = np.arccos(cos_theta_xy)
        angle_xy = np.degrees(theta_xy_rad)
    
    # 计算在xoz平面上的夹角
    denom_xz = np.sqrt(x**2 + z**2)
    if denom_xz == 0:
        angle_xz = np.nan  # 未定义
    else:
        cos_theta_xz = -x / denom_xz
        cos_theta_xz = np.clip(cos_theta_xz, -1.0, 1.0)
        theta_xz_rad = np.arccos(cos_theta_xz)
        angle_xz = np.degrees(theta_xz_rad)
    
    if y > 0:
        angle_xy = angle_xy * (-1)
    
    if z > 0:
        angle_xz = angle_xz * (-1)

    return angle_xy, angle_xz

def calculate_joints_angle(base_joints, base_index):
    temp_vector = base_joints[base_index+1] - base_joints[base_index]

    # 计算到各平面的旋转矩阵
    projections, rotation_matrices = vector_projections_and_rotation(temp_vector)
    print(np.dot(temp_vector, rotation_matrices["xoz"]))

    new_joints = rotate_point_cloud(base_joints,  rotation_matrices["xoz"])
    temp_vector_xoz = new_joints[base_index+1] - new_joints[base_index]

    # 计算到各平面的旋转矩阵
    projections, rotation_matrices = vector_projections_and_rotation(temp_vector_xoz)
    new_joints = rotate_point_cloud(new_joints,  rotation_matrices["xoy"])
    
    temp_vector = new_joints[base_index+2] - new_joints[base_index+1]
    angle_xy, angle_xz = calculate_angles_with_negative_x_axis(temp_vector)
    return angle_xy, angle_xz, new_joints

def calculate_joints_angle_mcp(base_joints, mcp_index):
    temp_vector = base_joints[mcp_index] - base_joints[0]

    # 计算到各平面的旋转矩阵
    projections, rotation_matrices = vector_projections_and_rotation(temp_vector)
    print(np.dot(temp_vector, rotation_matrices["xoz"]))

    new_joints = rotate_point_cloud(base_joints,  rotation_matrices["xoz"])
    # temp_vector_xoz = new_joints[mcp_index] - new_joints[0]

    # projections, rotation_matrices = vector_projections_and_rotation(temp_vector_xoz)
    # new_joints = rotate_point_cloud(new_joints,  rotation_matrices["xoy"])
    
    temp_vector = new_joints[mcp_index+1] - new_joints[mcp_index]
    angle_xy, angle_xz = calculate_angles_with_negative_x_axis(temp_vector)
    return angle_xy, angle_xz, new_joints

##########################################################################################################
def save_dict_to_json(data: Dict[str, Any], 
                     file_path: str, 
                     indent: int = 4, 
                     ensure_ascii: bool = False, 
                     sort_keys: bool = False) -> bool:
    """
    将Python字典保存为JSON文件
    
    参数:
        data: 要保存的字典数据
        file_path: JSON文件保存路径
        indent: 缩进空格数，默认为4（None表示不缩进）
        ensure_ascii: 是否确保ASCII字符，默认为False（支持中文）
        sort_keys: 是否对键进行排序，默认为False
    
    返回:
        bool: 成功返回True，失败返回False
    """
    try:
        # 确保目录存在
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        
        # 将字典写入JSON文件
        with open(file_path, 'w', encoding='utf-8') as json_file:
            json.dump(data, 
                     json_file, 
                     indent=indent, 
                     ensure_ascii=ensure_ascii, 
                     sort_keys=sort_keys)
        
        print(f"字典已成功保存至: {file_path}")
        return True
        
    except TypeError as e:
        print(f"类型错误: 字典中包含不可JSON序列化的对象 -> {e}")
        return False
    except IOError as e:
        print(f"文件读写错误: 无法写入文件 -> {e}")
        return False
    except Exception as e:
        print(f"未知错误: {e}")
        return False
def draw_joints_verts(joints, verts, faces):
    # 创建3D图形
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('MANO Hand Model with Corrected Joint Mapping')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 绘制关节点
    ax.scatter(joints[:, 0], joints[:, 1], joints[:, 2], s=30, c='b', depthshade=True)

    # 标注关节索引
    for i, joint in enumerate(joints):
        ax.text(joint[0], joint[1], joint[2], f'{i}', color='red', fontsize=9)

    # 绘制连杆结构
    segments = []
    for connection in connections:
        segments.append([joints[connection[0]], joints[connection[1]]])
    line_collection = Line3DCollection(segments, colors='k', linewidths=2.0, alpha=0.8)
    ax.add_collection3d(line_collection)

    # 绘制坐标系 (在16个关节位置)
    axis_length = 0.02  # 坐标系轴长度

    # for i in range(16):
    #     # 获取当前关节在21个关节点中的正确索引
    #     joint_idx = transform_to_joint_map[i]
    #     origin = joints[joint_idx]
        
    #     # 获取变换矩阵的旋转部分
    #     rotation = transforms_abs[i][:3, :3]
        
    #     # 计算坐标系三个轴的方向
    #     x_axis = origin + rotation[:, 0] * axis_length
    #     y_axis = origin + rotation[:, 1] * axis_length
    #     z_axis = origin + rotation[:, 2] * axis_length
        
    #     # 绘制坐标轴
    #     ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], 
    #             'r-', linewidth=2)  # X轴 (红色)
    #     ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], 
    #             'g-', linewidth=2)  # Y轴 (绿色)
    #     ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], 
    #             'b-', linewidth=2)  # Z轴 (蓝色)
        
    #     # 标注变换索引
    #     ax.text(origin[0], origin[1], origin[2], f'T{i}', color='purple', fontsize=8)

    # # 设置等比例坐标轴
    # max_val = np.max(np.abs(joints)) * 2
    # ax.set_xlim([-max_val, max_val])
    # ax.set_ylim([-max_val, max_val])
    # ax.set_zlim([-max_val, max_val])

    # # 添加图例说明
    # ax.text2D(0.05, 0.95, "Joint Indices (red) and Transform Indices (purple)", 
    #           transform=ax.transAxes, color='black')
    # ax.text2D(0.05, 0.92, "Coordinate Systems: X(Red), Y(Green), Z(Blue)", 
    #           transform=ax.transAxes, color='black')
    ######################################################################
    # # 绘制网格顶点
    # ax.scatter(verts[:, 0], verts[:, 1], verts[:, 2], 
    #            s=15, c='b', alpha=0.6, depthshade=True)

    # # 渲染三角网格
    # mesh = Poly3DCollection(verts[faces], alpha=0.3, linewidths=0.5, edgecolor='k')
    # mesh.set_facecolor([0.8, 0.8, 1.0])  # 浅蓝色表面
    # ax.add_collection3d(mesh)

    # 设置等比例坐标轴
    # max_val = np.max(np.abs(verts)) * 1.5
    max_val = np.max(verts)
    min_val = np.min(verts)
    ax.set_xlim([min_val, max_val])
    ax.set_ylim([min_val, max_val])
    ax.set_zlim([min_val, max_val])

    ######################################################################
    # 设置视角
    ax.view_init(elev=0, azim=-90)

    plt.tight_layout()
    plt.show()


# 定义16个变换到21个关节点的映射关系 [1,2](@ref)
transform_to_joint_map = [
    0,   # 手腕 -> 关节0
    5,   # 食指根部(MCP) -> 关节5
    6,   # 食指近端(PIP) -> 关节6
    7,   # 食指远端(DIP) -> 关节7
    9,   # 中指根部(MCP) -> 关节9
    10,  # 中指近端(PIP) -> 关节10
    11,  # 中指远端(DIP) -> 关节11
    13,  # 无名指根部(MCP) -> 关节13
    14,  # 无名指近端(PIP) -> 关节14
    15,  # 无名指远端(DIP) -> 关节15
    17,  # 小指根部(MCP) -> 关节17
    18,  # 小指近端(PIP) -> 关节18
    19,  # 小指远端(DIP) -> 关节19
    1,   # 拇指根部(CMC) -> 关节1
    2,   # 拇指近端(MCP) -> 关节2
    3    # 拇指远端(IP) -> 关节3
]
# transform_to_joint_map = [
#     0,   # 手腕 -> 关节0
#     1,   # 食指根部(MCP) -> 关节1
#     2,   # 食指近端(PIP) -> 关节2
#     3,   # 食指远端(DIP) -> 关节3
#     5,   # 中指根部(MCP) -> 关节5
#     6,  # 中指近端(PIP) -> 关节6
#     7,  # 中指远端(DIP) -> 关节7
#     9,  # 无名指根部(MCP) -> 关节9
#     10,  # 无名指近端(PIP) -> 关节10
#     11,  # 无名指远端(DIP) -> 关节11
#     13,  # 小指根部(MCP) -> 关节13
#     14,  # 小指近端(PIP) -> 关节14
#     15,  # 小指远端(DIP) -> 关节15
#     17,   # 拇指根部(CMC) -> 关节17
#     18,   # 拇指近端(MCP) -> 关节18
#     19    # 拇指远端(IP) -> 关节19
# ]
