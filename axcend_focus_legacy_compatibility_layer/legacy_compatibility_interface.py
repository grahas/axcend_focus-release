"""This module is used to simulate the legacy TCL interface."""
import base64
import json
import os
import random
import socket
import threading
import time
from datetime import datetime
from functools import wraps
from queue import Queue

import rclpy
from flask import Blueprint, Flask, jsonify, request
from flask_cors import CORS
from std_msgs.msg import String

from axcend_focus_legacy_compatibility_layer.legacy_compatibility_interface_node import \
    LegacyCompatibilityInterface
from axcend_focus_device_config import device_config

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})

legacy_compatibility_interface = None

# Simulating the system state and configuration
system_state = {
    "start_time": time.time(),
    "operator": None,
    "operator_ip": None,
    "reservation_key": None,
    "is_reserved": False,
    "firmware_UART_read_queue": Queue(),
    "firmware_UART_write_queue": Queue(),
}

# Create a blueprint for the RPC2 routes
rpc2_blueprint = Blueprint("rpc2", __name__)


def generate_key(user, ip):
    """Generate a key for the reservation system."""
    random_data = random.getrandbits(128)
    key_prefix = "FocusLC"
    key_suffix = (
        "Copyright 2019, Axcend LLC, All Rights Reserved."
        "I understand unauthorized use is illegal."
    )
    encoded_key = base64.b64encode(
        f"{random_data}:{key_prefix}:{user}:{ip}:{key_suffix}".encode()
    ).decode()
    return encoded_key


def update_legacy_compatibility_interface_reference(interface):
    """Update the reference to the legacy compatibility interface.

    This function is used to update the reference to the legacy compatibility interface
    in the test fixtures.
    """
    global legacy_compatibility_interface
    legacy_compatibility_interface = interface


def generate_json_response(method, result):
    """Generate a JSON response with the method, timestamp, microseconds, and result."""
    return jsonify(
        {
            "method": method,
            "timestamp": datetime.now().strftime("%a %b %d %H:%M:%S %z %Y"),
            "microseconds": str(int(time.time() * 1e6)),
            "result": result,
        }
    )


def check_key(func):
    """Check the key for the reservation system."""
    @wraps(func)
    def wrapper(*args, **kwargs):
        key = request.args.get("key")
        if not key:
            key = None

        if key != system_state["reservation_key"]:
            return generate_json_response("write", "INVALID KEY")

        return func(*args, **kwargs)

    return wrapper


@app.route("/")
def home():
    """Home route."""
    return "Hello, World!"


@rpc2_blueprint.route("/IsReserved", methods=["GET"])
def is_reserved():
    """Check if the system is reserved."""
    if system_state["is_reserved"]:
        result = {
            "status": "Reserved",
            "operator": system_state["operator"],
            "IP": system_state["operator_ip"],
        }
    else:
        result = {"status": "NOT RESERVED"}

    return generate_json_response("IsReserved", result)


@rpc2_blueprint.route("/ReserveSystem", methods=["GET"])
def reserve_system():
    """Reserve the system. User is sent as query parameter."""
    user = request.args.get("user")
    ip = request.remote_addr

    if system_state["is_reserved"]:
        info = {"status": "ALREADY RESERVED"}
    else:
        key = generate_key(user, ip)
        system_state.update(
            {
                "operator": user,
                "operator_ip": ip,
                "reservation_key": key,
                "is_reserved": True,
            }
        )
        info = {"status": "RESERVED", "key": key}

    return generate_json_response("ReserveSystem", info)


@rpc2_blueprint.route("/ReleaseSystem", methods=["GET"])
def release_system():
    """Release the system. Key is sent a query parameter."""
    key = request.args.get("key")

    if not system_state["is_reserved"]:
        status = "NOT RESERVED"
    elif key != system_state["reservation_key"]:
        status = "INVALID KEY"
    else:
        system_state.update(
            {
                "operator": None,
                "operator_ip": None,
                "reservation_key": None,
                "is_reserved": False,
            }
        )
        status = "OK"

    info = {"status": status}

    return generate_json_response("ReleaseSystem", info)


@rpc2_blueprint.route("/machinename", methods=["GET"])
def machine_name():
    """Return the hostname of the machine."""
    return generate_json_response("machinename", socket.gethostname())


@rpc2_blueprint.route("/version", methods=["GET"])
def version():
    """Return the version of the system."""
    return generate_json_response(
        "version", "Bridge 3.0.2\n Firmware 3.0.2\n Image 3.0.2\n Config 3.0.2\n"
    )


@rpc2_blueprint.route("/read", methods=["GET"])
@check_key
def read():
    """Return all the messages received from the firmware_UART_read topic."""
    # Concatenate and remove all the messages in the read queue
    results = ""
    while not system_state["firmware_UART_read_queue"].empty():
        results += system_state["firmware_UART_read_queue"].get()

    # Debug message for the results
    print("Read: ", results)
    return generate_json_response("read", results)


@rpc2_blueprint.route("/write", methods=["GET"])
@check_key
def write():
    """Write the message to the firmware_UART_write topic.

    The message is split into 32 character chunks and sent to the firmware.
    It will return the number of bytes sent to the firmware."""
    message = request.args.get("data")
    bytes_sent = 0
    if message is None:
        return generate_json_response("write", bytes_sent)

    # Split the message into 32 character chunks and write them to the topic
    for i in range(0, len(message), 32):
        msg = String()
        msg.data = message[i: i + 32]
        msg.data = "proto1 " + msg.data
        system_state["firmware_UART_write_queue"].put(msg)
        # Print the message to the console
        print(msg.data)
        bytes_sent += len(msg.data)

    return generate_json_response("write", bytes_sent)


@rpc2_blueprint.route("/cartridge_write", methods=["GET"])
@check_key
def cartridge_write():
    """This route is used to write the cartridge configuration to the firmware."""
    message = request.args.get("data")
    bytes_sent = 0
    if message is None:
        return generate_json_response("cartridge_write", bytes_sent)
    msg = String()
    msg.data = "cartridge_config " + message
    system_state["firmware_UART_write_queue"].put(msg)
    bytes_sent = len(msg.data)

    return generate_json_response("cartridge_write", bytes_sent)


@rpc2_blueprint.route("/cartridge_config", methods=["GET"])
def cartridge_config():
    """This route is used to read the cartridge configuration from the firmware."""
    results = legacy_compatibility_interface.request_cartridge_memory()
    return generate_json_response("cartridge_config", results)


@rpc2_blueprint.route("/machinestate", methods=["GET"])
def machine_state():
    """Return the machine state."""
    state_string = legacy_compatibility_interface.get_machine_state_string()
    return generate_json_response(
        "machinestate",
        {"state": state_string, "locked": "1" if system_state["is_reserved"] else "0"},
    )


@rpc2_blueprint.route("/clear", methods=["GET"])
@check_key
def clear():
    """Clear the serial port"""
    system_state["firmware_UART_read_queue"].queue.clear()
    system_state["firmware_UART_write_queue"].queue.clear()
    return generate_json_response("clear", "Cleared")


@rpc2_blueprint.route("/deviceconfig", methods=["GET"])
def deviceconfig():
    """Return the device configuration."""
    data = device_config.system_parameter_read_all()
    del data["OutputEvents"]
    return generate_json_response("deviceconfig", data)


@rpc2_blueprint.route("/update_system_parameters", methods=["GET"])
@check_key
def update_system_parameters():
    """Update the system parameters."""
    results = legacy_compatibility_interface.request_system_parameters_update()
    if results:
        return generate_json_response(
            "update_system_parameters", "UPDATED SYSTEM PARAMETERS"
        )

    return generate_json_response(
        "update_system_parameters", "FAILED TO UPDATE SYSTEM PARAMETERS"
    )


# Register the blueprint after defining the routes
app.register_blueprint(rpc2_blueprint, url_prefix="/RPC2")


def main():
    """Main function to start the Flask server and the ROS2 node."""
    rclpy.init()
    global legacy_compatibility_interface
    legacy_compatibility_interface = LegacyCompatibilityInterface(
        system_state["firmware_UART_write_queue"],
        system_state["firmware_UART_read_queue"],
    )

    flask_thread = threading.Thread(
        target=app.run, kwargs={"host": "0.0.0.0", "use_reloader": False, "port": 8000}
    )
    flask_thread.start()

    # Start the publish thread
    publish_thread = threading.Thread(
        target=legacy_compatibility_interface.publish_to_firmware_UART_write_thread
    )
    publish_thread.start()

    rclpy.spin(legacy_compatibility_interface)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
