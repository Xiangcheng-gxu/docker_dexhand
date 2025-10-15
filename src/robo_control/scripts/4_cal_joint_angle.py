import torch
import numpy as np
import math
import matplotlib.pyplot as plt
from manotorch.manolayer import ManoLayer, MANOOutput
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import open3d as o3d
from util import *
from scipy.spatial.transform import Rotation as R

if __name__ == "__main__":
    # 初始化MANO层
    ncomps = 45
    mano_layer = ManoLayer(use_pca=False, flat_hand_mean=False, ncomps=ncomps, side="right", center_idx=0)
    batch_size = 1
    shape = torch.zeros(batch_size, 10)
    pose = torch.zeros(batch_size, 3 + ncomps)

    pose = torch.tensor([[0.0, 0.0, 0.0, 0.052185188978910446, -0.2234778255224228, -0.6109187602996826, -0.2253139466047287, -0.025912242010235786, -0.34970590472221375, 0.10193215310573578, -0.13524684309959412, 0.055132463574409485, 0.05757101625204086, -0.09465328603982925, -0.5406275391578674, 0.014608566649258137, 0.028498828411102295, -0.11015968024730682, -0.015040257945656776, -0.0818653255701065, -0.22926290333271027, 0.31117913126945496, 0.11384628713130951, -0.3907489776611328, 0.18244653940200806, -0.09704270958900452, -0.21227005124092102, 0.1317605823278427, 0.0022792434319853783, -0.3182750344276428, 0.0810219794511795, -0.06335175037384033, -0.4405480921268463, 0.15712390840053558, -0.14265181124210358, -0.30338114500045776, 0.07242392003536224, 0.0540132001042366, -0.263729065656662, 0.1326579749584198, 0.1985720992088318, 0.21155454218387604, -0.01159933302551508, 0.12530238926410675, 0.025022679939866066, 0.021990125998854637, -0.005813572555780411, -0.01133244950324297]])
    shape = torch.tensor([[0.6993994116783142, -0.16909725964069366, -0.8955091834068298, -0.09764610230922699, 0.07754238694906235, 0.336286723613739, -0.05547792464494705, 0.5248727798461914, -0.38668063282966614, -0.00133091164752841]])

    mano_output: MANOOutput = mano_layer(pose, shape)
    verts = mano_output.verts.detach().numpy()[0]  # (778, 3)
    faces = mano_layer.th_faces.numpy()  # (1538, 3) 三角面片[1](@ref)

    joints = mano_output.joints.detach().numpy()[0]  # (21, 3)
    transforms_abs = mano_output.transforms_abs.detach().numpy()[0]  # (16, 4, 4)
    # print(transforms_abs)
    ######################################################################################
    # 1、mano旋转到初始位置
    mano_pose = pose.numpy()[0,0:3]
    mano_pose_matrix = R.from_euler('xyz', mano_pose, degrees=False).as_matrix()
    #******************************************************************************
    # rotation_matrix_y_180 = np.array([
    #     [-1,  0,  0],
    #     [ 0,  1,  0],
    #     [ 0,  0, -1]
    # ])
    # rotation_matrix_z_90 = np.array([
    #     [0, -1, 0],
    #     [1,  0, 0],
    #     [0,  0, 1]
    # ])
    # rotation_matrix_z_180 = np.array([
    #     [-1,  0,  0],
    #     [ 0, -1,  0],
    #     [ 0,  0,  1]
    # ])
    original_joints = rotate_point_cloud(joints, mano_pose_matrix)
    # original_joints = rotate_point_cloud(original_joints, rotation_matrix_z_180)
    #******************************************************************************
    # original_joints = rotate_point_cloud_inv(joints, mano_pose_matrix)
    # print(original_joints.shape)
    draw_joints_verts(original_joints, verts, faces)
    ######################################################################################
    # 2、解构拇指关节角
    # thumb_01 = original_joints[1] - original_joints[0]

    # # 计算到各平面的旋转矩阵
    # projections, rotation_matrices = vector_projections_and_rotation(thumb_01)
    # print(np.dot(thumb_01, rotation_matrices["xoz"]))

    # new_joints = rotate_point_cloud(original_joints,  rotation_matrices["xoz"])
    # thumb_01_xoz = new_joints[1] - new_joints[0]

    # # 计算到各平面的旋转矩阵
    # projections, rotation_matrices = vector_projections_and_rotation(thumb_01_xoz)
    # new_joints = rotate_point_cloud(new_joints,  rotation_matrices["xoy"])
    
    # temp_vector = new_joints[2] - new_joints[1]
    # angle_xy, angle_xz = calculate_angles_with_negative_x_axis(temp_vector)
    bone_12_angle_xy, bone_12_angle_xz, new_joints_12 = calculate_joints_angle(original_joints, 0)
    print(bone_12_angle_xy)
    print(bone_12_angle_xz)
    ######################################################################################
    bone_09 = original_joints[9] - original_joints[0]
    # 计算到各平面的旋转矩阵
    projections, rotation_matrices = vector_projections_and_rotation(bone_09)
    print(np.dot(bone_09, rotation_matrices["xoz"]))

    new_joints = rotate_point_cloud(original_joints,  rotation_matrices["xoz"])
    bone_09_xoz = new_joints[9] - new_joints[0]

    # 计算到各平面的旋转矩阵
    projections, rotation_matrices = vector_projections_and_rotation(bone_09_xoz)
    original_joints = rotate_point_cloud(new_joints,  rotation_matrices["xoy"])
    # draw_joints_verts(original_joints, verts, faces)
    ######################################################################################
    # 3、解构拇指其他关节角
    bone_23_angle_xy, bone_23_angle_xz, new_joints_23 = calculate_joints_angle(new_joints_12, 1)
    print(bone_23_angle_xy)
    print(bone_23_angle_xz)
    bone_34_angle_xy, bone_34_angle_xz, new_joints_34 = calculate_joints_angle(new_joints_23, 2)
    print(bone_34_angle_xy)
    print(bone_34_angle_xz)
    ######################################################################################
    # 4、解构食指的各关节角
    bone_56 = original_joints[6] - original_joints[5]
    # bone_56_angle_xy, bone_56_angle_xz = calculate_angles_with_negative_x_axis(bone_56)
    bone_56_angle_xy, bone_56_angle_xz, new_joints_56 = calculate_joints_angle_mcp(original_joints, 5)
    print(bone_56_angle_xy)
    print(bone_56_angle_xz)
    bone_67_angle_xy, bone_67_angle_xz, new_joints_67 = calculate_joints_angle(original_joints, 5)
    print(bone_67_angle_xy)
    print(bone_67_angle_xz)
    bone_78_angle_xy, bone_78_angle_xz, new_joints_78 = calculate_joints_angle(new_joints_67, 6)
    print(bone_78_angle_xy)
    print(bone_78_angle_xz)
    # draw_joints_verts(new_joints_78, verts, faces)
    ######################################################################################
    # 5、解构中指的各关节角
    bone_910 = original_joints[10] - original_joints[9]
    # bone_910_angle_xy, bone_910_angle_xz = calculate_angles_with_negative_x_axis(bone_910)
    bone_910_angle_xy, bone_910_angle_xz, new_joints_910 = calculate_joints_angle_mcp(original_joints, 9)
    draw_joints_verts(new_joints_56, verts, faces)
    print(bone_910_angle_xy)
    print(bone_910_angle_xz)
    bone_1011_angle_xy, bone_1011_angle_xz, new_joints_1011 = calculate_joints_angle(original_joints, 9)
    print(bone_1011_angle_xy)
    print(bone_1011_angle_xz)
    bone_1112_angle_xy, bone_1112_angle_xz, new_joints_1112 = calculate_joints_angle(new_joints_1011, 10)
    print(bone_1112_angle_xy)
    print(bone_1112_angle_xz)
    # draw_joints_verts(new_joints_1112, verts, faces)
    ######################################################################################
    # 6、解构无名指的各关节角
    bone_1314 = original_joints[14] - original_joints[13]
    # bone_1314_angle_xy, bone_1314_angle_xz = calculate_angles_with_negative_x_axis(bone_1314)
    bone_1314_angle_xy, bone_1314_angle_xz, new_joints_1314 = calculate_joints_angle_mcp(original_joints, 13)
    print(bone_1314_angle_xy)
    print(bone_1314_angle_xz)
    bone_1415_angle_xy, bone_1415_angle_xz, new_joints_1415 = calculate_joints_angle(original_joints, 13)
    print(bone_1415_angle_xy)
    print(bone_1415_angle_xz)
    bone_1516_angle_xy, bone_1516_angle_xz, new_joints_1516 = calculate_joints_angle(new_joints_1415, 14)
    print(bone_1516_angle_xy)
    print(bone_1516_angle_xz)
    # draw_joints_verts(new_joints_1516, verts, faces)
    ######################################################################################
    # 6、解构小拇指的各关节角
    bone_1718 = original_joints[18] - original_joints[17]
    # bone_1718_angle_xy, bone_1718_angle_xz = calculate_angles_with_negative_x_axis(bone_1718)
    bone_1718_angle_xy, bone_1718_angle_xz, new_joints_1718 = calculate_joints_angle_mcp(original_joints, 17)
    print(bone_1718_angle_xy)
    print(bone_1718_angle_xz)
    bone_1819_angle_xy, bone_1819_angle_xz, new_joints_1819 = calculate_joints_angle(original_joints, 17)
    print(bone_1819_angle_xy)
    print(bone_1819_angle_xz)
    bone_1920_angle_xy, bone_1920_angle_xz, new_joints_1920 = calculate_joints_angle(new_joints_1819, 18)
    print(bone_1920_angle_xy)
    print(bone_1920_angle_xz)
    # draw_joints_verts(new_joints_1920, verts, faces)
    prefix = ['rh']
    joint_angle = {prefix[0] + '_FFJ4': bone_56_angle_xz, prefix[0] + '_FFJ3': bone_56_angle_xy, prefix[0] + '_FFJ2': bone_67_angle_xy, prefix[0] + '_FFJ1': bone_78_angle_xy, \
            prefix[0] + '_MFJ4': bone_910_angle_xz, prefix[0] + '_MFJ3': bone_910_angle_xy, prefix[0] + '_MFJ2': bone_1011_angle_xy, prefix[0] + '_MFJ1': bone_1112_angle_xy, \
            prefix[0] + '_RFJ4': - bone_1314_angle_xz, prefix[0] + '_RFJ3': bone_1314_angle_xy, prefix[0] + '_RFJ2': bone_1415_angle_xy, prefix[0] + '_RFJ1': bone_1516_angle_xy, \
            prefix[0] + '_LFJ4': - bone_1718_angle_xz, prefix[0] + '_LFJ3': bone_1718_angle_xy, prefix[0] + '_LFJ2': bone_1819_angle_xy, prefix[0] + '_LFJ1': bone_1920_angle_xy, \
            prefix[0] + '_THJ4': bone_12_angle_xz, prefix[0] + '_THJ3': bone_12_angle_xy, prefix[0] + '_THJ2': bone_23_angle_xy, prefix[0] + '_THJ1': bone_34_angle_xy}
    print(joint_angle)
    save_dict_to_json(joint_angle, "./joint_angle.json")