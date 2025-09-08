import yaml
from pathlib import Path


def load_robot_config(file_path: str = 'utils/robot_config.yaml') -> dict:
    """
    Load the robot configuration from a YAML file.

    Args:
        file_path (str): Path to the YAML configuration file.

    Returns:
        dict: The robot configuration.

    Raises:
        FileNotFoundError: If the YAML file does not exist.
    """
    path = Path(file_path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {file_path}")

    with path.open('r') as file:
        return yaml.safe_load(file) or {}


def parse_robot_config(name: str, file_path: str = 'utils/robot_config.yaml') -> dict:
    """
    Parse the robot configuration to a more accessible format.

    Args:
        name (str): The name or alias of the robot.
        file_path (str): Path to the YAML configuration file.

    Returns:
        dict: Parsed robot configuration with keys as robot name or alias.
    """
    config = load_robot_config(file_path)
    parsed_config = {}

    name = name.replace(" ", "_")

    # Search for the robot by name or alias
    for robot in config.get('robots', []):
        for key in ('name', 'alias'):
            candidate = robot.get(key, '').replace(" ", "_")
            if candidate == name:
                # check required fields
                for field in ('ip', 'type', 'prompts'):
                    if field not in robot or robot[field] in (None, ''):
                        raise ValueError(f"Robot '{name}' is missing required field: {field}")

                parsed_config[candidate] = {
                    'ip': robot['ip'],
                    'type': robot['type'],
                    'prompts': robot['prompts']
                }

    return parsed_config
