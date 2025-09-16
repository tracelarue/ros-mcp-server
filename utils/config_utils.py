import yaml
from pathlib import Path


def load_robot_config(robot_name: str, specs_dir: str) -> dict:
    """
    Load the robot configuration from a YAML file by robot name.

    Args:
        robot_name (str): The name of the robot.
        specs_dir (str): Directory containing robot specification files.

    Returns:
        dict: The robot configuration.

    Raises:
        FileNotFoundError: If the YAML file does not exist.
    """
    robot_name = robot_name.replace(" ", "_")
    file_path = Path(specs_dir) / f"{robot_name}.yaml"
    
    if not file_path.exists():
        raise FileNotFoundError(f"Robot config file not found: {file_path}")

    with file_path.open('r') as file:
        return yaml.safe_load(file) or {}


def parse_robot_config(name: str, specs_dir: str = 'utils/robot_specifications') -> dict:
    """
    Parse the robot configuration to a more accessible format.

    Args:
        name (str): The name of the robot.
        specs_dir (str): Directory containing robot specification files.

    Returns:
        dict: Parsed robot configuration with robot name as key.
    """
    config = load_robot_config(name, specs_dir)
    parsed_config = {}

    name = name.replace(" ", "_")

    # Check if the loaded config has the required fields
    if not config:
        raise ValueError(f"No configuration found for robot '{name}'")

    # Check required fields
    for field in ('type', 'prompts'):
        if field not in config or config[field] in (None, ''):
            raise ValueError(f"Robot '{name}' is missing required field: {field}")

    # Create configuration with robot name as key
    parsed_config[name] = {
        'type': config['type'],
        'prompts': config['prompts']
    }

    return parsed_config
