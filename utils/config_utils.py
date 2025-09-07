import yaml

def load_robot_config() -> dict:
    """
    Load the robot configuration from a YAML file.

    Returns:
        dict: The robot configuration.
    """
    with open('utils/robot_config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config