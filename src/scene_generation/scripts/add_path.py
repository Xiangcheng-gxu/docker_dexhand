import os
import xml.etree.ElementTree as ET
from pathlib import Path

def clean_xml_content(raw_content):
    """
    预处理XML内容：确保XML声明位于字符串的绝对开头。
    去除XML声明前的所有空白字符（包括BOM）。
    """
    # 查找 <?xml 声明的位置
    start_pos = raw_content.find('<?xml')
    if start_pos == -1:
        # 如果找不到声明，可能文件格式特殊，返回原内容
        return raw_content
    if start_pos > 0:
        # 如果声明不在开头，则去除声明前的所有字符
        print(f"检测到XML声明前存在{start_pos}个字符，正在清理...")
        return raw_content[start_pos:]
    # 如果声明已经在开头，则直接返回
    return raw_content

def update_urdf_mesh_paths_recursive(root_directory, ros_workspace_src_path):
    """
    递归地更新指定目录及其所有子目录中URDF文件的mesh文件路径为package://格式
    
    参数:
        root_directory: 包含URDF文件（可能位于子目录）的根目录路径
        ros_workspace_src_path: ROS工作空间的src目录绝对路径
    """
    # 统计变量
    processed_files = 0
    updated_meshes = 0
    
    # 确保ROS工作空间src路径是绝对路径
    ros_src_path = Path(ros_workspace_src_path).absolute()
    print(f"ROS工作空间src目录: {ros_src_path}")
    
    # 遍历根目录及其所有子目录
    for root_dir, dirs, files in os.walk(root_directory):
        print(f"\n扫描目录: {root_dir}")
        
        for filename in files:
            if filename.endswith('.urdf'):
                urdf_path = os.path.join(root_dir, filename)
                print(f"处理文件: {urdf_path}")
                
                try:
                    # 计算从ROS工作空间src目录到当前URDF文件所在目录的相对路径
                    urdf_dir_path = Path(root_dir).absolute()
                    
                    # 确保URDF目录在ROS工作空间src目录下
                    if ros_src_path not in urdf_dir_path.parents:
                        print(f"警告: {urdf_dir_path} 不在ROS工作空间src目录下")
                        continue
                    
                    # 计算相对路径（从src目录开始）
                    relative_path = urdf_dir_path.relative_to(ros_src_path)
                    # 将路径转换为ROS package格式（使用正斜杠）
                    package_relative_path = str(relative_path).replace('\\', '/')
                    print(f"Package相对路径: {package_relative_path}")
                    
                    # 【关键步骤】先读取文件内容并进行清理
                    with open(urdf_path, 'r', encoding='utf-8') as f:
                        raw_content = f.read()
                    
                    # 清理XML内容（去除开头的空白字符等）
                    cleaned_content = clean_xml_content(raw_content)
                    
                    # 使用清理后的内容进行解析
                    try:
                        root = ET.fromstring(cleaned_content)
                    except ET.ParseError as e:
                        print(f"XML解析失败 {filename}: {e}")
                        # 尝试使用更宽松的方式处理
                        try:
                            # 移除可能的BOM字符
                            if cleaned_content.startswith('\ufeff'):
                                cleaned_content = cleaned_content[1:]
                            root = ET.fromstring(cleaned_content)
                        except ET.ParseError as e2:
                            print(f"无法修复的XML解析错误: {e2}")
                            continue
                    
                    updated_count = 0
                    # 查找所有mesh元素（包括visual和collision）
                    for mesh in root.findall('.//mesh'):
                        current_filename = mesh.get('filename')
                        
                        # 检查是否是相对路径的STL文件
                        if current_filename and current_filename.endswith('.stl'):
                            # 提取STL文件名
                            stl_filename = os.path.basename(current_filename)
                            # 构建新的package://路径
                            new_path = f"package://{package_relative_path}/{stl_filename}"
                            # 更新filename属性
                            mesh.set('filename', new_path)
                            updated_count += 1
                            updated_meshes += 1
                            print(f"  更新: {current_filename} -> {new_path}")
                    
                    # 将修改后的XML树转换为字符串
                    rough_string = ET.tostring(root, encoding='utf-8').decode('utf-8')
                    # 添加XML声明并格式化
                    cleaned_string = '<?xml version="1.0" encoding="utf-8"?>\n' + rough_string
                    
                    # 保存修改后的URDF文件
                    with open(urdf_path, 'w', encoding='utf-8') as f:
                        f.write(cleaned_string)
                    
                    processed_files += 1
                    print(f"成功更新 {filename}: 修改了 {updated_count} 个mesh路径")
                    
                except ET.ParseError as e:
                    print(f"XML解析错误 {filename}: {e}")
                except Exception as e:
                    print(f"处理 {filename} 时出错: {e}")
    
    print(f"\n处理完成! 共处理 {processed_files} 个URDF文件，更新了 {updated_meshes} 个mesh路径")

def verify_urdf_changes(root_directory):
    """
    验证URDF文件中的mesh路径是否已修改为正确的package://格式
    """
    print("\n开始验证URDF文件修改...")
    
    for root_dir, dirs, files in os.walk(root_directory):
        for filename in files:
            if filename.endswith('.urdf'):
                urdf_path = os.path.join(root_dir, filename)
                
                try:
                    with open(urdf_path, 'r', encoding='utf-8') as f:
                        content = f.read()
                    
                    # 清理内容后再解析
                    cleaned_content = clean_xml_content(content)
                    root = ET.fromstring(cleaned_content)
                    
                    for mesh in root.findall('.//mesh'):
                        filename_attr = mesh.get('filename')
                        if filename_attr and 'package://' in filename_attr:
                            print(f"✓ {filename}: 已修改为 {filename_attr}")
                        elif filename_attr:
                            print(f"✗ {filename}: 未修改 - {filename_attr}")
                            
                except Exception as e:
                    print(f"验证 {filename} 时出错: {e}")

# 使用示例
if __name__ == "__main__":
    for i in ['contactdb','egadb','kit','ycb']:
        root_dir = f'/home/user/workspace/src/scene_generation/object/{i}'
    
    # 设置ROS工作空间的src目录绝对路径
        ros_src_path = '/home/user/workspace/src'
        
        print("开始批量更新URDF文件路径...")
        print("=" * 50)
        
        # 执行更新
        update_urdf_mesh_paths_recursive(root_dir, ros_src_path)
        
        print("\n" + "=" * 50)
        print("开始验证修改结果...")
        print("=" * 50)
        
        # 验证修改
        verify_urdf_changes(root_dir)
        
        print("\n所有操作完成!")