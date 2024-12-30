import yaml
import math
import xml.etree.ElementTree as ET
# Define a custom constructor for the !Rpy tag
def rpy_constructor(loader, node):
    # Parse the Rpy node as a dictionary
    rpy_data = loader.construct_mapping(node)
    return rpy_data

# Register the constructor with the !Rpy tag
yaml.add_constructor('!Rpy', rpy_constructor, Loader=yaml.FullLoader)

# Function to get translation and rotation for a given frame name
def get_robot_pose(robot_name, yaml_file):
    # Load the YAML file
    # yaml_file = "Auto_calibration/scripts/models/disposable_line/disposable_line.yaml"
    with open(yaml_file, "r") as file:
        data = yaml.load(file, Loader=yaml.FullLoader)
    
    # Iterate over directives to find the specified frame
    for directive in data.get("directives", []):
        if "add_frame" in directive and directive["add_frame"]["name"] == robot_name:
            translation = directive["add_frame"]["X_PF"].get("translation")
            rotation = directive["add_frame"]["X_PF"].get("rotation")["deg"]
            rotation = [math.radians(angle) for angle in rotation]
            return {"translation": translation, "rotation": rotation}
    
    # If frame not found, return None
    return None

def get_box_pose(box_name, urdf_file):
    # Parse the XML file
    tree = ET.parse(urdf_file)

    root = tree.getroot()
    
    # Iterate over link elements to find the specified box
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            if visual.get('name') == box_name:
                origin = visual.find('origin')
                geometry = visual.find('geometry/box')
                
                rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
                xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
                size = [float(x) for x in geometry.get('size', '0 0 0').split()]
                
                return {
                    "rpy": rpy,
                    "xyz": xyz,
                    "size": size
                }
    
    # If the box is not found, return None
    return None

# Usage example
# box_info = get_box_pose("robot1_box1")

# print("RPY:", box_info["rpy"])
# print("XYZ:", box_info["xyz"])
# print("Box Size:", box_info["size"])

# frame_info = get_robot_pose("robot3_origin")
# print("Translation:", frame_info["translation"])
# print("Rotation:", frame_info["rotation"])