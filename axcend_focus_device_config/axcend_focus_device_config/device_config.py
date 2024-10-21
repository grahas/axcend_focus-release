import json
import os
import yaml

system_parameters_file_path = os.environ.get("SYS_PARAMS_FILE")
config_file_path = os.environ.get("CONFIG_FILE_PATH")

def get_config() -> dict:
    """Read the config.yaml file."""
    # Read the config.yaml file
    with open(config_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)

def system_parameter_read_key(key: str):
    """Read a key from the system parameters file."""
    # Open the file and load the JSON object
    with open(system_parameters_file_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # Return the value of the key
    return data[key]

def system_parameter_read_all() -> dict:
    """Read the entire system parameters file."""
    # Open the file and load the JSON object
    with open(system_parameters_file_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # Return the data
    return data

def main():
    # Check if the file exists
    if not os.path.exists(os.environ.get("SYS_PARAMS_FILE")):
        print("The system parameters file does not exist.")
    
if __name__ == '__main__':
    main()
