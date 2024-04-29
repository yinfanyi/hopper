import xml.etree.ElementTree as ET
import re
from typing import Optional,Dict
import numpy as np
import matplotlib.pyplot as plt
# 增加需求 在excel中读入数据

class Tensegrity:
    def __init__(self, xml_path:str, Nodes:np.ndarray, Cb_in:Optional[np.ndarray]=None, Cs_in:Optional[np.ndarray]=None):
        self.xml_path = xml_path
        self.tree = None
        self.root = None
        self.rod_list = []
        self.string_list = []
        self.N = Nodes

        if Cb_in is not None:
            self.C_b = self.tenseg_ind2C(Cb_in)
        else:
            self.C_b = None
        if Cs_in is not None:
            self.C_s = self.tenseg_ind2C(Cs_in)
        else:
            self.C_s = None

    def init(self):
        self.load_xml()
        self.fill_rod_list()
        self.fill_string_list()
    
    # 优化代码格式，写完xml后，运行该函数，使xml文件更易读
    def prettify(self, elem, level=0):
        indent = "\n" + level*"  "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = indent + "  "
            if not elem.tail or not elem.tail.strip():
                elem.tail = indent
            for elem in elem:
                self.prettify(elem, level+1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = indent
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = indent

    # 生成杆和杆的各类属性，记录在self.rod_element列表里
    def generate_rod_element(self, rod_name:str, pos:str, **kwargs):
        match = re.search(r"rod(\d+)_(\d+)", rod_name)
        assert match, "rod_name必须符合rodx_y的格式"
        # 检查 rod_name 对应的 rod 是否已经存在
        for rod in self.rod_list:
            if rod['name'] == rod_name:
                print(f"Rod with name '{rod_name}' already exists in rod_list.")
                return
        rod_dict = {'name':rod_name, 'pos':pos}
        rod_dict.update(kwargs)
        self.rod_list.append(rod_dict)

    # 生成绳索的各类属性，记录在self.string_list列表里
    def generate_string_element(self, from_site_name:str, to_site_name:str, **kwargs):
        match1 = re.search(r"s(\d+)", from_site_name)
        assert match1, "from_site_name必须符合sx的格式"
        match2 = re.search(r"s(\d+)", to_site_name)
        assert match2, "to_site_name必须符合sx的格式"

        x_str = match1.group(1)
        y_str = match2.group(1)
        x_str = str(x_str)
        y_str = str(y_str)
        string_name = f'td{x_str}_{y_str}'

        # 检查 string_name 对应的 string 是否已经存在
        for string in self.string_list:
            if string['name'] == string_name:
                print(f"String with name '{string_name}' already exists in string_list.")
                return
        string_dict = {'name':string_name, 'from_site_name':from_site_name, 'to_site_name':to_site_name}
        string_dict.update(kwargs)
        self.string_list.append(string_dict)

    # 填充rod_list，使用self.N和Cb、C
    def fill_rod_list(self):
        if self.C_b is None or self.C_s is None or self.N is None:
            print('数据不完全')
            return False
        
        B = np.dot(self.N, self.C_b.T)
        bar_start_nodes = np.array([np.squeeze(self.N[:, self.C_b[j, :] == -1]) for j in range(B.shape[1])]).T
        bar_end_nodes = np.array([np.squeeze(self.N[:, self.C_b[j, :] == 1]) for j in range(B.shape[1])]).T

        for j in range(B.shape[1]):
            fromto_coords=bar_start_nodes[:, j].tolist() + bar_end_nodes[:, j].tolist()
            
            midpoint_coords = [(fromto_coords[0] + fromto_coords[3]) / 2, 
                    (fromto_coords[1] + fromto_coords[4]) / 2, 
                    (fromto_coords[2] + fromto_coords[5]) / 2]
            
            midpoint_coords_str = "{} {} {}".format(midpoint_coords[0], 
                                                    midpoint_coords[1], 
                                                    midpoint_coords[2])
            
            fromto_coords_str = "{} {} {} {} {} {}".format(fromto_coords[0], 
                                            fromto_coords[1], 
                                            fromto_coords[2], 
                                            fromto_coords[3], 
                                            fromto_coords[4], 
                                            fromto_coords[5])
            from_point_coords_str =  "{} {} {}".format(fromto_coords[0], 
                                            fromto_coords[1], 
                                            fromto_coords[2])
            to_point_coords_str =  "{} {} {}".format(fromto_coords[3], 
                                                    fromto_coords[4], 
                                                    fromto_coords[5])
        

            x_str= str(np.argmax(self.C_b[j, :] == -1))
            y_str = str(np.argmax(self.C_b[j, :] == 1))
            self.generate_rod_element(body_name=f'rod{x_str}_{y_str}', body_pos=midpoint_coords_str, 
                                      joint_type='free', joint_name=f'joint{x_str}_{y_str}',
                                      geom_name=f'geom{x_str}_{y_str}', geom_type='cylinder', 
                                      geom_fromto=fromto_coords_str, geom_density='1000', geom_size='0.014',
                                      site1_name = 's'+x_str, site1_pos=from_point_coords_str,
                                      site2_name = 's'+y_str, site2_pos=to_point_coords_str)
    
    # 填充string_list，使用self.N和Cb、C
    def fill_string_list(self):
        if self.C_b is None or self.C_s is None or self.N is None:
            print('数据不完全')
            return False
        
        S = np.dot(self.N, self.C_s.T)
        for j in range(S.shape[1]):
            self.generate_string_element(from_site_name=f's{np.argmax(self.C_s[j, :] == -1)}', 
                                        to_site_name=f's{np.argmax(self.C_s[j, :] == 1)}')
        
    # 将rod_list内容加载到xmltree里，注意，tree在类里，还需要tree.write将内容加载到xml文件
    def load_rod_list2xmltree(self):
        self.remove_all_body_from_xml()
        for rod_element in self.rod_list:
            self.load_rod_element2xmltree(rod_element)

    # 将string_list写道xmltree里
    def load_string_list2xmltree(self):
        self.remove_all_tendon_from_xml()
        for string_element in self.string_list:
            self.load_string_element2xmltree(string_element)

    # 将杆和绳加到xml tree里
    def load_tensegrity2xmltree(self):
        self.load_rod_list2xmltree()
        self.load_string_list2xmltree()

    # 将在rod_list中的其中一项rod_element:Dict写进xml
    def load_rod_element2xmltree(self, rod_element:Dict):
        new_body = ET.Element('body')
        new_body.set('name', rod_element.get('name'))
        new_body.set('pos', rod_element.get('pos'))

        joint = ET.Element('joint')
        for key, value in rod_element.items():
            if key.startswith('joint_'):
                joint.set(key.split('_')[-1], value)

        geom = ET.Element('geom')
        geom.set('name', rod_element.get('geom_name'))
        geom.set('type', rod_element.get('geom_type'))
        geom.set('fromto', rod_element.get('geom_fromto'))
        geom.set('density', rod_element.get('geom_density'))
        geom.set('size', rod_element.get('geom_size'))

        site1 = ET.Element('site')
        site1.set('name', rod_element.get('site1_name'))
        site1.set('pos', rod_element.get('site1_pos'))

        site2 = ET.Element('site')
        site2.set('name', rod_element.get('site2_name'))
        site2.set('pos', rod_element.get('site2_pos'))

        new_body.append(joint)
        new_body.append(geom)
        new_body.append(site1)
        new_body.append(site2)
        # 将新的<body>子元素添加到<worldbody>的最后
        worldbody = self.root.find('worldbody')
        worldbody.append(new_body)
        
    # 将在string_list中的一项string_element写进xml
    def load_string_element2xmltree(self, string_element:Dict):
        new_spatial = ET.Element('spatial')
        new_spatial.set('name', string_element.get('name'))

        site1 = ET.Element('site')
        site1.set('site', string_element.get('from_site_name'))

        site2 = ET.Element('site')
        site2.set('site', string_element.get('to_site_name'))

        new_spatial.append(site1)
        new_spatial.append(site2)

        tendon = self.root.find('tendon')
        tendon.append(new_spatial)

    # 指定杆名字和两端坐标，在文件中生成杆件
    def generate_rod_element2xml(self, rod_name:str, fromto_coords, mass:Optional[str]=None, rod_radius='0.014'):
        assert len(fromto_coords) == 6 and all(isinstance(coord, (int, float)) for coord in fromto_coords), "fromto_coords必须是包含六个数字的列表"
        match = re.search(r"rod(\d+)_(\d+)", rod_name)
        assert match, "rod_name必须符合rodx_y的格式"
        assert mass is None or (isinstance(mass, str) and mass.isdigit()), "mass must be a string representing a number"
        
        x_str = match.group(1)
        y_str = match.group(2)
        x_str = str(x_str)
        y_str = str(y_str)
        
        fromto_coords_str = "{} {} {} {} {} {}".format(fromto_coords[0], 
                                                       fromto_coords[1], 
                                                       fromto_coords[2], 
                                                       fromto_coords[3], 
                                                       fromto_coords[4], 
                                                       fromto_coords[5])
        from_point_coords_str =  "{} {} {}".format(fromto_coords[0], 
                                                   fromto_coords[1], 
                                                   fromto_coords[2])
        to_point_coords_str =  "{} {} {}".format(fromto_coords[3], 
                                                 fromto_coords[4], 
                                                 fromto_coords[5])
        
        midpoint_coords = [(fromto_coords[0] + fromto_coords[3]) / 2, 
                    (fromto_coords[1] + fromto_coords[4]) / 2, 
                    (fromto_coords[2] + fromto_coords[5]) / 2]
        midpoint_coords_str = "{} {} {}".format(midpoint_coords[0], 
                                                midpoint_coords[1], 
                                                midpoint_coords[2])
        
        new_body = ET.Element('body')
        new_body.set('name', rod_name)
        new_body.set('pos', midpoint_coords_str)

        joint = ET.Element('joint')
        joint.set('type', 'free')
        joint.set('name', 'joint' + x_str + '_' + y_str)

        geom = ET.Element('geom')
        geom.set('name', 'geom' + x_str + '_' + y_str)
        geom.set('type', 'cylinder')
        geom.set('fromto', fromto_coords_str)
        if mass is not None:
            geom.set('mass', mass)
        else:
            geom.set('density', '1000')
        geom.set('size', rod_radius)

        site1 = ET.Element('site')
        site1.set('name', 's' + x_str)
        site1.set('pos', from_point_coords_str)

        site2 = ET.Element('site')
        site2.set('name', 's' + y_str)
        site2.set('pos', to_point_coords_str)

        new_body.append(joint)
        new_body.append(geom)
        new_body.append(site1)
        new_body.append(site2)
        # 将新的<body>子元素添加到<worldbody>的最后
        worldbody = self.root.find('worldbody')
        worldbody.append(new_body)

    # 指定绳索两端的点，在文件中生成绳索
    def generate_tendon_object2xml(self, from_site_name:str, 
                               to_site_name:str, 
                               stiffness:Optional[str]=None):
        match1 = re.search(r"s(\d+)", from_site_name)
        assert match1, "from_site_name必须符合sx的格式"
        match2 = re.search(r"s(\d+)", to_site_name)
        assert match2, "to_site_name必须符合sx的格式"
        assert stiffness is None or (isinstance(stiffness, str) and 
                                     stiffness.isdigit()), "stiffness must be a string representing a number"

        x_str = match1.group(1)
        y_str = match2.group(1)
        x_str = str(x_str)
        y_str = str(y_str)

        new_spatial = ET.Element('spatial')
        new_spatial.set('name', 'td' + x_str + '_' + y_str)

        if stiffness is not None:
            new_spatial.set('stiffness', stiffness)

        site1 = ET.Element('site')
        site1.set('site', from_site_name)

        site2 = ET.Element('site')
        site2.set('site', to_site_name)

        new_spatial.append(site1)
        new_spatial.append(site2)

        tendon = self.root.find('tendon')
        tendon.append(new_spatial)

    # 删除指定的物体
    def remove_body_from_xml(self, body_name):
        for body in self.root.iter('body'):
            if body.get('name') == body_name:
                for worldbody in self.root.iter('worldbody'):
                    worldbody.remove(body)
                print(f"成功移除名称为{body_name}的body元素及其所有子元素。")
                return
        print(f"未找到名称为{body_name}的body元素。")

    # 删除所有的物体
    def remove_all_body_from_xml(self):
        worldbody = self.root.find('worldbody')
        bodies = list(worldbody.iter('body'))
        for body in bodies:
            worldbody.remove(body)

    # 删除指定的绳索
    def remove_tendon_from_xml(self, spatial_name):
        for spatial in self.root.iter('spatial'):
            if spatial.get('name') == spatial_name:
                tendon = self.root.find('tendon')
                tendon.remove(spatial)
                print(f"成功移除名称为{spatial_name}的绳索元素。")
                return
        print(f"未找到名称为{spatial_name}的body元素。")

    # 删除所有的绳索
    def remove_all_tendon_from_xml(self):
        tendon = self.root.find('tendon')
        spatials = list(tendon.iter('spatial'))
        for spatial in spatials:
            tendon.remove(spatial)

    # 使连接的杆件对矩阵转换为连接矩阵
    def tenseg_ind2C(self, C_ind):
        """
        Creates a connectivity matrix from input index notation array and node matrix.

        Inputs:
            C_ind: index connectivity array (m x 2 array for m members)
            Nodes: node matrix (3 x n array for n nodes)

        Outputs:
            C_mat: connectivity matrix (m x n matrix satisfying M = N*C')

        Example: If given four nodes (N is a 3x4 matrix), and you want one bar to
        be the vector from node 1 to 3, and another bar from node 2 to 4, input
        C_ind would be: C_ind = np.array([[1, 3], [2, 4]])

        C_b = tenseg_ind2C(np.array([[1, 3], [2, 4]]), N)
        """
        nmembers = C_ind.shape[0]  # Number of members being created
        n = self.N.shape[1]  # Number of nodes in the structure

        # Initialize connectivity matrix
        C_mat = np.zeros((n, nmembers))

        for i in range(nmembers):  # Go through each member
            # Get indices for start and end points
            side1 = C_ind[i, 0]
            side2 = C_ind[i, 1]

            # Put -1 at index for start point, 1 at index for end point
            C_mat[side1 - 1, i] = -1
            C_mat[side2 - 1, i] = 1

        return C_mat.T

    # 使用matplotlib画出张拉整体结构
    def tenseg_plot(self, show_labels=False):
        BarWidth = 2
        StringWidth = 1
        NodeSize = 25

        # Get difference between min and max values for each axis
        fig_out = plt.figure()
        ax = fig_out.add_subplot(111, projection='3d')

        # Plot bar member vectors
        if self.C_b is not None:
            B = np.dot(self.N, self.C_b.T)
            bar_start_nodes = np.zeros((3, B.shape[1]))
            bar_end_nodes = np.zeros((3, B.shape[1]))
            for j in range(B.shape[1]):
                bar_start_nodes[:,j] = np.squeeze(self.N[:, self.C_b[j,:] == -1])
                bar_end_nodes[:,j] = np.squeeze(self.N[:, self.C_b[j,:] == 1])

            for j in range(B.shape[1]):
                ax.plot([bar_start_nodes[0,j], bar_end_nodes[0,j]], 
                        [bar_start_nodes[1,j], bar_end_nodes[1,j]], 
                        [bar_start_nodes[2,j], bar_end_nodes[2,j]], 
                        color='black', linewidth=BarWidth)
                if show_labels:
                    ax.text((bar_start_nodes[0,j] + bar_end_nodes[0,j])/2, 
                            (bar_start_nodes[1,j] + bar_end_nodes[1,j])/2, 
                            (bar_start_nodes[2,j] + bar_end_nodes[2,j])/2, 
                            str(j+1), color='black')
            # ax.quiver(bar_start_nodes[0,:], bar_start_nodes[1,:], bar_start_nodes[2,:], B[0,:], B[1,:], B[2,:], color='black', linewidth=BarWidth)

        # Plot string member vectors
        if self.C_s is not None:
            S = np.dot(self.N, self.C_s.T)
            string_start_nodes = np.zeros((3, S.shape[1]))
            string_end_nodes = np.zeros((3, S.shape[1]))
            for j in range(S.shape[1]):
                string_start_nodes[:,j] = np.squeeze(self.N[:, self.C_s[j,:] == -1])
                string_end_nodes[:,j] = np.squeeze(self.N[:, self.C_s[j,:] == 1])
            for j in range(S.shape[1]):
                ax.plot([string_start_nodes[0,j], string_end_nodes[0,j]], 
                        [string_start_nodes[1,j], string_end_nodes[1,j]], 
                        [string_start_nodes[2,j], string_end_nodes[2,j]], 
                        color='red', linewidth=StringWidth)
                if show_labels:
                    ax.text((string_start_nodes[0,j] + string_end_nodes[0,j])/2, 
                            (string_start_nodes[1,j] + string_end_nodes[1,j])/2, 
                            (string_start_nodes[2,j] + string_end_nodes[2,j])/2, 
                            str(j+1), color='red')  # Add string labels

        ax.scatter(self.N[0,:], self.N[1,:], self.N[2,:], color='blue', s=NodeSize)
        if show_labels:
            for i in range(self.N.shape[1]):
                ax.text(self.N[0,i], self.N[1,i], self.N[2,i], str(i+1), color='blue')  # Add node labels
        
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        plt.show()
        # ax.quiver(string_start_nodes[0,:], string_start_nodes[1,:], string_start_nodes[2,:], S[0,:], S[1,:], S[2,:], color='red', linestyle='-', linewidth=StringWidth)

    # 使所有节点整体移动一个距离
    def move_nodes(self, x, y, z):
        self.N = self.N + np.array([[x], [y], [z]])

    # 加载xml到类中
    def load_xml(self):
        self.tree = ET.parse(self.xml_path)
        self.root = self.tree.getroot()

    # 将张拉整体结构输出到xml文件中
    def load_tensegrity_to_xml_old(self):
        self.remove_all_body_from_xml()
        self.remove_all_tendon_from_xml()

        if len(self.C_b) > 0:
            B = np.dot(self.N, self.C_b.T)
            bar_start_nodes = np.array([np.squeeze(self.N[:, self.C_b[j, :] == -1]) for j in range(B.shape[1])]).T
            bar_end_nodes = np.array([np.squeeze(self.N[:, self.C_b[j, :] == 1]) for j in range(B.shape[1])]).T

            for j in range(B.shape[1]):
                self.generate_rod_element2xml(rod_name=f'rod{np.argmax(self.C_b[j, :] == -1)}_{np.argmax(self.C_b[j, :] == 1)}', 
                                          fromto_coords=bar_start_nodes[:, j].tolist() + bar_end_nodes[:, j].tolist())

        if len(self.C_s) > 0:
            S = np.dot(self.N, self.C_s.T)
            for j in range(S.shape[1]):
                self.generate_tendon_object2xml(from_site_name=f's{np.argmax(self.C_s[j, :] == -1)}', 
                                            to_site_name=f's{np.argmax(self.C_s[j, :] == 1)}')
        self.prettify(self.root)
        self.tree.write(self.xml_path, encoding='utf-8', xml_declaration=True)

    # 修改指定元素的参数
    def modify_body_element(self, name:str, *args):
        worldbody = self.root.find('worldbody')
        body = worldbody.find(f"./body[@name='{name}']")
        if body is not None:
            if len(args) == 3:  # modify_body_element(name, attribute, value)
                attribute, value = args
                if len(body.findall(attribute)) > 0:
                    item = body.find(attribute)
                    item.text = value
                    print(f"Modified {attribute} for body {name} to {value}")
                else:
                    body.set(attribute, value)
                    print(f"Added {attribute} for body {name} with value {value}")
            elif len(args) == 4:  # modify_body_element(name, element_name, attribute, value)
                element_name, attribute, value = args
                for child in body:
                    if child.tag == element_name:
                        child.set(attribute, value)
                        print(f"Modified {attribute} for {element_name} in body {name} to {value}")
                        break
                else:
                    new_element = ET.Element(element_name)
                    new_element.set(attribute, value)
                    body.append(new_element)
                    print(f"Added {element_name} with {attribute} for body {name} with value {value}")
            self.prettify(self.root)
            self.tree.write(self.xml_path, encoding='utf-8', xml_declaration=True)
            return
        print(f"Body {name} not found in XML")


# 直接写xml三杆六棱柱张拉整体结构
def main1():
    xml_path = "./xml/TT_prism.xml"
    N = np.array([[0.5, 0, 0], [0, 0.866, 0], [-0.5, 0, 0], [0.5, 0, 1], [0, 0.866, 1], [-0.5, 0, 1]]).T
    Cb_in = np.array([[3, 5], [1, 6], [2, 4]])  # Bar 1 connects node 3 to 5, etc
    Cs_in = np.array([[1, 2], [2, 3], [3, 1], [4, 5], [5, 6], [6, 4], [1, 4], [2, 5], [3, 6]])  # String one is node 1 to 2
    prism = Tensegrity(xml_path, N, Cb_in, Cs_in)
    prism.load_tensegrity_to_xml_old()
    prism.tenseg_plot(True)

# 直接写xml六杆superball张拉整体结构
def main2():
    xml_path = "./xml/TT_six_bars.xml"
    # N = np.array([[0.5, 0, 0], [0, 0.866, 0], [-0.5, 0, 0], [0.5, 0, 1], [0, 0.866, 1], [-0.5, 0, 1]]).T
    phi = (1 + 5 ** 0.5) / 2
    temp = np.array([[0, -1, phi], [0, -1, -phi], [0, 1, phi], [0, 1, -phi]]) / (2 * phi)
    N = np.concatenate((temp, temp[:, [2, 0, 1]], temp[:, [1, 2, 0]], [[0, 0, 0]]), axis=0).T
    N = N[:, [1, 3, 5, 7, 9, 11, 0, 2, 4, 6, 8, 10, 12]]

    Cb_in = np.array([[1, 7], [2, 8], [3, 9], 
                      [4, 10], [5, 11], [6, 12]])  
    
    Cs_in = np.array([[1, 3], [1, 5], [1, 6], [1, 9],
                      [2, 3], [2, 9], [2, 11], [2, 12], 
                      [3, 5], [3, 11],
                      [4, 5], [4, 7], [4, 8], [4, 11],
                      [5, 7], 
                      [6, 7], [6, 9], [6, 10],
                      [7, 10], 
                      [8, 10], [8, 11], [8, 12],
                      [9, 12], 
                      [10, 12]])  
    super_ball = Tensegrity(xml_path, N, Cb_in, Cs_in)
    super_ball.move_nodes(0, 0, 1) # 使之整体向上移动1m，避免碰到地板
    super_ball.load_tensegrity_to_xml_old()
    super_ball.modify_body_element('rod1_7','pos','99 99 99')
    # super_ball.tenseg_plot(True)

# 使用rod_list写xml
def main3():
    xml_path = "./xml/TT_six_bars.xml"
    # N = np.array([[0.5, 0, 0], [0, 0.866, 0], [-0.5, 0, 0], [0.5, 0, 1], [0, 0.866, 1], [-0.5, 0, 1]]).T
    phi = (1 + 5 ** 0.5) / 2
    temp = np.array([[0, -1, phi], [0, -1, -phi], [0, 1, phi], [0, 1, -phi]]) / (2 * phi)
    N = np.concatenate((temp, temp[:, [2, 0, 1]], temp[:, [1, 2, 0]], [[0, 0, 0]]), axis=0).T
    N = N[:, [1, 3, 5, 7, 9, 11, 0, 2, 4, 6, 8, 10, 12]]

    Cb_in = np.array([[1, 7], [2, 8], [3, 9], 
                      [4, 10], [5, 11], [6, 12]])  
    
    Cs_in = np.array([[1, 3], [1, 5], [1, 6], [1, 9],
                      [2, 3], [2, 9], [2, 11], [2, 12], 
                      [3, 5], [3, 11],
                      [4, 5], [4, 7], [4, 8], [4, 11],
                      [5, 7], 
                      [6, 7], [6, 9], [6, 10],
                      [7, 10], 
                      [8, 10], [8, 11], [8, 12],
                      [9, 12], 
                      [10, 12]])  
    
    super_ball = Tensegrity(xml_path, N, Cb_in, Cs_in)
    super_ball.move_nodes(0, 0, 1) # 使之整体向上移动1m，避免碰到地板
    super_ball.init()
    super_ball.load_tensegrity2xmltree()
    # super_ball.load_string_element2xmltree(super_ball.string_list[0])
    super_ball.prettify(super_ball.root)
    super_ball.tree.write(super_ball.xml_path, encoding='utf-8', xml_declaration=True)

if __name__ == "__main__":
    # main1()
    # main2()
    main3()