import os
import numpy as np
import trimesh
import xml.etree.ElementTree as ET
from pathlib import Path

def fix_xml_format(urdf_path):
    """
    修复URDF文件的XML格式，确保XML声明位于文件开头
    """
    with open(urdf_path, 'r') as f:
        content = f.read()
    
    # 查找XML声明的位置
    xml_declaration = '<?xml version="1.0"?>'
    declaration_pos = content.find(xml_declaration)
    
    if declaration_pos > 0:
        # 如果XML声明不在文件开头，修复它
        content = content[declaration_pos:] + content[:declaration_pos]
        
        # 移除XML声明前的所有内容
        with open(urdf_path, 'w') as f:
            f.write(content)
        print(f"已修复 {urdf_path} 的XML格式")

def calculate_inertial_properties(stl_path):
    """
    计算STL文件的惯性属性（质量、质心和惯性张量）
    使用更可靠的方法计算惯性属性
    """
    try:
        # 加载STL文件并进行处理
        mesh = trimesh.load(stl_path)
        
        # 确保网格是水密的（watertight）
        if not mesh.is_watertight:
            print(f"警告: {stl_path} 不是水密网格，尝试修复...")
            # 尝试修复网格
            mesh.fill_holes()
            mesh.process(validate=True)
            
            # 如果仍然不是水密的，使用凸包近似
            if not mesh.is_watertight:
                print(f"无法修复 {stl_path}，使用凸包近似")
                mesh = mesh.convex_hull
        
        # 计算体积（立方米）
        volume = mesh.volume
        
        # 检查体积是否合理
        if volume < 1e-10:  # 非常小的体积
            print(f"警告: {stl_path} 的体积非常小 ({volume:.6e})，使用边界框估算")
            # 使用边界框估算体积
            bounds = mesh.bounds
            size = bounds[1] - bounds[0]
            volume = size[0] * size[1] * size[2]
        
        # 假设密度为1000 kg/m³（类似水的密度）
        density = 1000.0
        mass = density * volume
        
        # 获取质心（在网格坐标系中）
        centroid = mesh.center_mass
        
        # 计算惯性张量（关于质心）
        inertia_tensor = mesh.moment_inertia
        
        # 检查惯性张量是否全为零
        if np.allclose(inertia_tensor, 0, atol=1e-10):
            print(f"警告: {stl_path} 的惯性张量全为零，使用近似计算")
            # 使用边界框估算惯性张量
            bounds = mesh.bounds
            size = bounds[1] - bounds[0]
            
            # 确保尺寸不为零
            if any(dim < 1e-6 for dim in size):
                print(f"警告: {stl_path} 的尺寸过小，使用默认惯性张量")
                # 使用默认惯性张量
                inertia_tensor = np.diag([0.1, 0.1, 0.1])
            else:
                # 对于长方体，惯性张量的近似计算
                # Ixx = (m/12) * (h^2 + d^2), 其中h和d是垂直于x轴的尺寸
                ixx = (mass / 12) * (size[1]**2 + size[2]**2)
                iyy = (mass / 12) * (size[0]**2 + size[2]**2)
                izz = (mass / 12) * (size[0]**2 + size[1]**2)
                
                inertia_tensor = np.array([[ixx, 0, 0],
                                           [0, iyy, 0],
                                           [0, 0, izz]])
        
        return mass, centroid, inertia_tensor
    except Exception as e:
        print(f"计算惯性属性时出错: {e}")
        # 返回合理的默认值
        print(f"使用默认惯性属性 for {stl_path}")
        return 1.0, [0.0, 0.0, 0.0], np.diag([0.1, 0.1, 0.1])

def add_inertial_to_urdf(urdf_path, stl_path):
    """
    为URDF文件添加惯性属性并精确控制格式化
    """
    try:
        # 先修复XML格式
        fix_xml_format(urdf_path)
        
        # 读取原始文件内容
        with open(urdf_path, 'r') as f:
            original_content = f.read()
        
        # 解析URDF文件
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        
        # 找到link元素
        link = root.find('link')
        if link is None:
            print(f"在 {urdf_path} 中未找到link元素")
            return
        
        # 检查是否已存在inertial元素
        if link.find('inertial') is not None:
            print(f"{urdf_path} 中已存在inertial元素，跳过处理")
            return
        
        # 计算惯性属性
        mass, centroid, inertia_tensor = calculate_inertial_properties(stl_path)
        
        # 创建格式化的inertial元素字符串
        formatted_inertial = f'''
    <inertial>
      <mass value="{mass:.6f}" />
      <origin xyz="{centroid[0]:.6f} {centroid[1]:.6f} {centroid[2]:.6f}" />
      <inertia ixx="{inertia_tensor[0, 0]:.6f}" ixy="{inertia_tensor[0, 1]:.6f}" ixz="{inertia_tensor[0, 2]:.6f}" 
               iyy="{inertia_tensor[1, 1]:.6f}" iyz="{inertia_tensor[1, 2]:.6f}" izz="{inertia_tensor[2, 2]:.6f}" />
    </inertial>'''
        
        # 在原始内容中找到</collision>标签的位置
        collision_end_pos = original_content.find('</collision>')
        if collision_end_pos == -1:
            print(f"在 {urdf_path} 中未找到</collision>标签")
            return
        
        # 计算</collision>标签后的位置
        insertion_pos = collision_end_pos + len('</collision>')
        
        # 获取</collision>后的内容，检查是否需要添加换行
        after_collision = original_content[insertion_pos:]
        
        # 在</collision>后插入格式化的inertial元素
        # 确保在</collision>和<inertial>之间有适当的换行
        if after_collision.startswith('\n'):
            # 如果已经有换行，直接插入
            new_content = (original_content[:insertion_pos] + 
                          formatted_inertial + 
                          original_content[insertion_pos:])
        else:
            # 如果没有换行，先添加换行再插入
            new_content = (original_content[:insertion_pos] + 
                          '\n' + formatted_inertial + 
                          original_content[insertion_pos:])
        
        # 保存修改后的URDF文件
        with open(urdf_path, 'w', encoding='utf-8') as f:
            f.write(new_content)
        
        print(f"已为 {urdf_path} 添加格式化的惯性属性")
        print(f"  质量: {mass:.6f} kg")
        print(f"  质心: [{centroid[0]:.6f}, {centroid[1]:.6f}, {centroid[2]:.6f}]")
        print(f"  惯性张量: \n{inertia_tensor}")
        
    except ET.ParseError as e:
        print(f"解析 {urdf_path} 时出错: {e}")
        print("请手动检查该文件的XML格式")
    except Exception as e:
        print(f"处理 {urdf_path} 时发生未知错误: {e}")

def process_ycb_dataset(dataset_path):
    """
    处理YCB数据集中的所有物体
    """
    dataset_path = Path(dataset_path)
    
    # 遍历所有物体文件夹
    for object_dir in dataset_path.iterdir():
        if object_dir.is_dir():
            # 查找URDF和STL文件
            urdf_file = None
            stl_file = None
            
            for file in object_dir.iterdir():
                if file.suffix == '.urdf':
                    urdf_file = file
                elif file.suffix == '.stl':
                    stl_file = file
            
            # 如果找到URDF和STL文件，则处理它们
            if urdf_file and stl_file:
                print(f"处理物体: {object_dir.name}")
                add_inertial_to_urdf(urdf_file, stl_file)
            else:
                print(f"在 {object_dir} 中未找到URDF或STL文件")

if __name__ == "__main__":
    # 指定数据集路径
    datasets = ['contactdb', 'egadb', 'graspdb', 'ycb']
    
    for dataset_name in datasets:
        dataset_path = f'/home/user/workspace/src/scene_generation/object copy 2/{dataset_name}'  
        if os.path.exists(dataset_path):
            print(f"处理 {dataset_name} 数据集...")
            process_ycb_dataset(dataset_path)
            print(f"{dataset_name}数据集的所有物体的惯性属性已添加并格式化完成！")
        else:
            print(f"指定的路径 {dataset_path} 不存在，跳过处理。")