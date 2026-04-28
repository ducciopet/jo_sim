import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch.actions import SetLaunchConfiguration


def inject_plugins(context):
    """Replace plugins in the world file with those from worlds/plugins.sdf, write to a temp file."""
    world_path = context.launch_configurations['world']
    plugins_path = os.path.join(
        get_package_share_directory('jo_sim'), 'worlds', 'plugins.sdf'
    )

    tree = ET.parse(world_path)
    root = tree.getroot()

    world_elem = root.find('world')
    if world_elem is None:
        raise RuntimeError(f"No <world> element in {world_path}")

    for plugin in list(world_elem.findall('plugin')):
        world_elem.remove(plugin)

    plugins_root = ET.parse(plugins_path).getroot()
    for i, plugin in enumerate(plugins_root.findall('plugin')):
        world_elem.insert(i, plugin)

    with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False, prefix='jo_world_') as tmp:
        tmp.write('<?xml version="1.0" ?>\n')
        tree.write(tmp, encoding='unicode', xml_declaration=False)
        tmp_path = tmp.name

    return [SetLaunchConfiguration('world', tmp_path)]
