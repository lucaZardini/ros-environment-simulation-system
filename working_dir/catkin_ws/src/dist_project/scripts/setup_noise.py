import yaml
import os
from lxml import etree


quadrotor_sensor_file = "../../hector_quadrotor_noetic/hector_quadrotor/hector_quadrotor_gazebo/urdf/quadrotor_sensors.gazebo.xacro"
sonar_sensor_file = "../../hector_quadrotor_noetic/hector_models/hector_sensors_description/urdf/sonar_sensor.urdf.xacro"


def load_yaml_config(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)


def update_xacro_file(xml_file, config):
    # Define the correct namespace
    namespace_uri = "http://ros.org/wiki/xacro"
    nsmap = {"xacro": namespace_uri}

    # Parse the XML file
    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(xml_file, parser)
    root = tree.getroot()

    # Update plugin parameters
    for plugin in root.xpath(".//plugin"):
        name = plugin.get("name").lower()
        if 'imu' in name and 'imu' in config['sensors']:
            update_plugin_params(plugin, config['sensors']['imu'])
        elif 'baro' in name and 'barometer' in config['sensors']:
            update_plugin_params(plugin, config['sensors']['barometer'])
        elif 'gps' in name and 'gps' in config['sensors']:
            update_plugin_params(plugin, config['sensors']['gps'])
        elif 'magnetic' in name and 'magnetic' in config['sensors']:
            update_plugin_params(plugin, config['sensors']['magnetic'])
        elif 'p3d' in name and 'p3d' in config['sensors']:
            update_plugin_params(plugin, config['sensors']['p3d'])

    # Save the updated file
    base_dir = os.path.dirname(xml_file)
    updated_file = os.path.join(base_dir, os.path.basename(xml_file))

    # Write XML back with correct namespaces
    with open(updated_file, "wb") as f:
        f.write(etree.tostring(tree, pretty_print=True, xml_declaration=True, encoding="UTF-8"))

    print(f"Updated file saved at: {updated_file}")


def update_sonar_file(xml_file, config):
    # Define the correct namespace
    namespace_uri = "http://ros.org/wiki/xacro"
    nsmap = {"xacro": namespace_uri}

    # Parse the XML file
    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(xml_file, parser)
    root = tree.getroot()

    # Update plugin parameters
    for plugin in root.xpath(".//plugin"):
        name = plugin.get("name").lower()
        if 'gazebo_ros_${name}_controller' in name and 'sonar' in config['sensors']:
            update_plugin_params(plugin, config['sensors']['sonar'])

    # Save the updated file
    base_dir = os.path.dirname(xml_file)
    updated_file = os.path.join(base_dir, os.path.basename(xml_file))

    # Write XML back with correct namespaces
    with open(updated_file, "wb") as f:
        f.write(etree.tostring(tree, pretty_print=True, xml_declaration=True, encoding="UTF-8"))

    print(f"Updated file saved at: {updated_file}")


# Helper function to update plugin parameters
def update_plugin_params(plugin, params):
    for key, value in params.items():
        tag = plugin.find(key)
        if tag is not None:
            if isinstance(value, str):
                value = value.replace(',', '')
            tag.text = f"{value}"


if __name__ == "__main__":
    # get current directory
    current_dir = os.path.dirname(os.path.realpath(__file__))
    print(f"Current directory: {current_dir}")
    yaml_file = "../params/drone_simulation.yaml"
    yaml_file = os.path.join(current_dir, yaml_file)

    sonar_file = sonar_sensor_file
    sonar_file = os.path.join(current_dir, sonar_file)
    xml_file = quadrotor_sensor_file
    xml_file = os.path.join(current_dir, xml_file)

    config = load_yaml_config(yaml_file)
    update_xacro_file(xml_file, config)
    update_sonar_file(sonar_file, config)
    # print(f"Updated {xml_file} based on {yaml_file}")
