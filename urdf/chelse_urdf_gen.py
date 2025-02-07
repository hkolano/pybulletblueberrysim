import xml.etree.ElementTree as ET
import csv

round_limit = 5



CSV_FILE = f'/home/chelse/Documents/research/WholeBranchBush6Branch2urdfSingleValues.csv'

# Read data from the CSV file
with open(CSV_FILE, 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip the header row
    for row in reader:
        density_value = float(row[0])
        mass_value = float(round(float(row[1]),round_limit))
        radius_value = float(round(float(row[2]),round_limit))
        length_value = float(row[3])

        density_str = str(density_value)
        mass_str = str(mass_value)
        radius_str = str(radius_value)
        length_str = str(length_value)

print('radius ', radius_value)
print('mass ', mass_value)
print('length ', length_value)
print('density ', density_value)

base_link_mass = 100000
base_link_radius = 0.5
base_link_length = 0.05


# Function to get inertial values
def get_ixx_or_iyy(m_val, r_val, l_val):
    ixx_iyy = (m_val/12 * (3* r_val **2 + l_val**2)) + (m_val * (0.5* l_val)**2)
    return str(round(ixx_iyy,round_limit))

def get_izz(m_val, r_val):
    izz = 0.5 * m_val * r_val**2
    return str(round(izz, round_limit*2))


def create_link(name, type='cylinder', m_str='1', m_val=1, r_val=0.1, l_val=1, shape_dim=[1, 0.1], origin=[0, 0, 0, 0, 0, 0], material='gray', color=[0.5, 0.5, 0.5, 1], collision=True, inertial=False):
    """
    Creates a new link element.

    :param name: Name of the link.
    :param type: Type of link shape ('cylinder' or 'box')
    :param origin: Origin of the joint (x y z rx ry rz)
    :param length: Length of the link.
    :param radius: Radius of the link.
    :return: A new link element.
    """
    link = ET.Element('link', name=name)
    
    if inertial:
        if type == 'cylinder':
            inertial = ET.SubElement(link, 'inertial')
            mass = ET.SubElement(inertial, 'mass', value=m_str)
            inertia = ET.SubElement(inertial, 'inertia',
                                    ixx=get_ixx_or_iyy(m_val, r_val, l_val), 
                                    ixy='0.0', ixz='0.0',
                                    iyy=get_ixx_or_iyy(m_val, r_val, l_val), 
                                    iyz='0.0', 
                                    izz=get_izz(m_val, r_val))
        else: 
            inertial = ET.SubElement(link, 'inertial')
            mass = ET.SubElement(inertial, 'mass', value='1.0')
            inertia = ET.SubElement(inertial, 'inertia',
                                    ixx='0.0', ixy='0.0', ixz='0.0',
                                    iyy='0.0', iyz='0.0', izz='0.0')
    
    visual = ET.SubElement(link, 'visual')
    visual_origin = ET.SubElement(visual, 'origin', xyz=' '.join(map(str, origin[:3])), rpy=' '.join(map(str, origin[3:])))
    geometry = ET.SubElement(visual, 'geometry')
    material = ET.SubElement(visual, 'material', name=material)
    color = ET.SubElement(material, 'color', rgba=' '.join(map(str, color)))

    if collision:
        collision = ET.SubElement(link, 'collision')
        collision_origin = ET.SubElement(collision, 'origin', xyz=' '.join(map(str, origin[:3])), rpy=' '.join(map(str, origin[3:])))
        collision_geometry = ET.SubElement(collision, 'geometry')

    if type == 'cylinder':
        cylinder = ET.SubElement(geometry, 'cylinder', length=str(shape_dim[0]), radius=str(shape_dim[1]))

        if collision:
            collision_cylinder = ET.SubElement(collision_geometry, 'cylinder', length=str(shape_dim[0]), radius=str(shape_dim[1]))

    if type == 'box':
        box = ET.SubElement(geometry, 'box', size=' '.join(map(str, shape_dim)))
        if collision:
            collision_box = ET.SubElement(collision_geometry, 'box', size=' '.join(map(str, shape_dim)))

    if type == 'sphere':
        sphere = ET.SubElement(geometry, 'sphere', radius=' '.join(map(str, shape_dim)))
        if collision:
            collision_sphere = ET.SubElement(collision_geometry, 'sphere', radius=' '.join(map(str, shape_dim)))
    
    return link

def create_joint(name, parent, child, origin=[0, 0, 0, 0, 0, 0], joint_type='revolute', axis='0 0 1', limit=[-1.57, 1.57], effort=10, velocity=1):
    """
    Creates a new joint element.

    :param name: Name of the joint.
    :param parent: Name of the parent link.
    :param child: Name of the child link.
    :param origin: Origin of the joint (x y z rx ry rz)
    :param joint_type: Type of the joint (default is 'revolute').
    :param axis: Axis of the joint (default is '0 0 1').
    :param limit: Lower and upper limit of the joint [lower, upper]
    :param effort: Effort of the joint
    :param velocity: Velocity of the joint
    :return: A new joint element.
    """
    joint = ET.Element('joint', name=name, type=joint_type)
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    ET.SubElement(joint, 'axis', xyz=axis)
    ET.SubElement(joint, 'origin', xyz=' '.join(map(str, origin[:3])), rpy=' '.join(map(str, origin[3:])))
    
    if joint_type in ['revolute', 'prismatic']:
        ET.SubElement(joint, 'limit', lower=str(limit[0]), upper=str(limit[1]), effort=str(effort), velocity=str(velocity))
    
    return joint

def save_urdf(root, file_path):
    """
    Saves the URDF tree to a file.

    :param root: The root element of the URDF tree.
    :param file_path: The file path to save the URDF.
    """
    tree = ET.ElementTree(root)
    tree.write(file_path, xml_declaration=True, encoding='utf-8', method="xml")


def create_branch(filename, robot_name,link_shape='cylinder'):
    # Create the URDF
    robot = ET.Element('robot', name=robot_name)
    base_link = create_link(
        'base_link', link_shape, 
        str(base_link_mass), 
        base_link_mass, 
        base_link_radius, 
        base_link_length, 
        [base_link_length, base_link_radius], 
        origin=[0, 0, 0, 0, 0, 0], 
        material='blue', 
        color=[0, 0, 1, 1],
        collision=True, 
        inertial=True
    )
   
    base_branch_joint = create_joint(
        name='base_branch_joint', 
        parent='base_link',  
        child='branch_link', 
        origin=[0, 0, base_link_length, 0, 0, 0],  
        joint_type='revolute', 
        axis='0 1 0', 
        limit=[-1.57, 1.57], 
        effort=10, 
        velocity=1.0
    )

    # Create branch_link directly
    branch_link = create_link(
        'branch_link', link_shape, 
        str(mass_value), 
        mass_value, 
        radius_value, 
        length_value, 
        [length_value, radius_value], 
        origin=[0, 0, base_link_length/2, 0, 0, 0],  
        material='green', 
        color=[0, 1, 0, 1], 
        collision=True, 
        inertial=True
    )
   
    robot.append(base_link)
    robot.append(base_branch_joint)
    robot.append(branch_link)

    # Save the URDF to a file
    save_urdf(robot, f'./urdf/{filename}.urdf')

# # Example usage to generate a planar manipulator
create_branch('chelse_bush6_branch2', 'blueberry_plant')

# # Example usage to generate a manipulator with n-DOF and specified axes of rotation
# axes = ['0 0 1', '0 0 1', '1 0 0', '1 0 0', '0 1 0', '1 0 0']
# shape_dims = [[0.5, 0.05], [0, 0], [0.5, 0.05], [0.5, 0.05], [0.5, 0.05], [0.5, 0.05]]
# create_manipulator('auto_gen_manip', 'new_robot', axes=axes, shape_dims=shape_dims)