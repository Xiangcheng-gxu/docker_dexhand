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
        # 加载STL文件
        mesh = trimesh.load(stl_path)

        # 强制使用凸包进行计算，这对于非水密网格更稳健
        mesh = mesh.convex_hull
        
        # 确保网格有体积
        if mesh.volume < 1e-9:
            print(f"警告: {stl_path} 的凸包体积过小 ({mesh.volume:.3e})。将使用边界框近似。")
            # 使用边界框估算
            bounds = mesh.bounds
            size = bounds[1] - bounds[0]
            # 避免尺寸为零
            size = np.maximum(size, 1e-6)
            mesh.volume = size[0] * size[1] * size[2]
            # 对于边界框，质心就是中心
            mesh.center_mass = mesh.bounds.mean(axis=0)
            # 重新创建一个基于边界框的trimesh对象以计算惯性
            mesh = trimesh.creation.box(extents=size, transform=trimesh.transformations.translation_matrix(mesh.center_mass))

        # 假设一个更合理的平均密度，例如 800 kg/m^3
        # YCB物体密度各不相同，这是一个折衷值
        density = 800.0
        mesh.density = density
        
        # 从trimesh获取惯性属性
        mass = mesh.mass
        centroid = mesh.center_mass
        inertia_tensor = mesh.moment_inertia

        # 关键修复：检查惯性张量的对角线元素是否过小
        # 如果惯性张量接近于零，则基于边界框进行估算
        min_inertia = 1e-7
        if np.any(np.diag(inertia_tensor) < min_inertia) or np.allclose(inertia_tensor, 0):
            print(f"警告: {stl_path} 计算出的惯性张量过小或为零，使用边界框近似。")
            bounds = mesh.bounds
            size = bounds[1] - bounds[0]
            size = np.maximum(size, 1e-6) # 避免尺寸为零
            
            # 对于长方体，惯性张量的近似计算
            ixx = (mass / 12) * (size[1]**2 + size[2]**2)
            iyy = (mass / 12) * (size[0]**2 + size[2]**2)
            izz = (mass / 12) * (size[0]**2 + size[1]**2)
            
            # 确保对角线元素不小于最小值
            ixx = max(ixx, min_inertia)
            iyy = max(iyy, min_inertia)
            izz = max(izz, min_inertia)

            inertia_tensor = np.diag([ixx, iyy, izz])

        return mass, centroid, inertia_tensor
    except Exception as e:
        print(f"计算惯性属性时出错: {e}")
        # 返回合理的默认值
        print(f"使用默认惯性属性 for {stl_path}")
        return 0.1, [0.0, 0.0, 0.0], np.diag([1e-3, 1e-3, 1e-3])

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
        dataset_path = f'/home/user/workspace/src/scene_generation/object/{dataset_name}'  
        if os.path.exists(dataset_path):
            print(f"处理 {dataset_name} 数据集...")
            process_ycb_dataset(dataset_path)
            print(f"{dataset_name}数据集的所有物体的惯性属性已添加并格式化完成！")
        else:
            print(f"指定的路径 {dataset_path} 不存在，跳过处理。")