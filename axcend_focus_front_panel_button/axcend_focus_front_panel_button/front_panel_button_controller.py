import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from evdev import InputDevice, categorize, ecodes, list_devices
import threading

SHORT_BUTTON_THRESHOLD = 2


def execute_color(red: int, green: int, blue: int):
    """Set the color of the front panel button."""
    colors = ["red", "green", "blue"]
    values = [red, green, blue]

    for color in colors:
        base_path = "/sys/class/leds/front-panel:{color}/brightness"
        with open(base_path.format(color=color), "w", encoding="utf-8") as f:
            f.write(str(values[colors.index(color)]))


def set_color(option):
    colors = {
        "red": (255, 0, 0),
        "blue": (0, 0, 255),
        "green": (0, 255, 0),
        "cyan": (0, 255, 255),
        "magenta": (255, 0, 255),
        "white": (255, 255, 255),
    }
    if option in colors:
        execute_color(*colors[option])


def find_button_devices():
    devices = [InputDevice(path) for path in list_devices()]
    button_devices = []
    for device in devices:
        if "front_panel_button" in device.name.lower():
            button_devices.append(device)
    return button_devices


class FrontPanelButtonController(Node):
    def __init__(self):
        super().__init__("front_panel_button_controller")
        self.device_phase_subscription = self.create_subscription(
            String, "device_phase", self.device_phase_callback, 10
        )
        self.publisher_ = self.create_publisher(String, "front_panel_button", 10)
        self.button_devices = find_button_devices()
        if self.button_devices:
            self.listen_thread = threading.Thread(target=self.listen_to_buttons)
            self.listen_thread.start()
        else:
            self.get_logger().info("No front panel button devices found.")

    def device_phase_callback(self, msg):
        if msg.data == "Idle":
            set_color("green")
        elif msg.data == "Refill":
            set_color("cyan")
        elif msg.data == "Pressurize":
            set_color("cyan")
        elif msg.data == "Equilibriate":
            set_color("cyan")
        elif msg.data == "Run":
            set_color("blue")
        elif msg.data == "Finalize":
            set_color("white")
        elif msg.data == "Service":
            set_color("blue")
        elif msg.data == "Abort":
            set_color("mageenta")

    def listen_to_buttons(self):
        button_press_times = {}
        for device in self.button_devices:
            self.get_logger().info(f"Listening on {device.name}...")
            for event in device.read_loop():
                if event.type == ecodes.EV_KEY:
                    key_event = categorize(event)
                    if key_event.keystate == key_event.key_down:
                        # Record the time when the button is pressed
                        button_press_times[key_event.keycode] = time.time()
                    elif key_event.keystate == key_event.key_up:
                        # Calculate the duration of the press
                        if key_event.keycode in button_press_times:
                            press_duration = (
                                time.time() - button_press_times[key_event.keycode]
                            )
                            button_press_msg = String()
                            # Say if it was a short or long press
                            if press_duration < SHORT_BUTTON_THRESHOLD:
                                button_press_msg.data = f"Button {key_event.keycode} short pressed for {press_duration:.2f} seconds"
                            else:
                                button_press_msg.data = f"Button {key_event.keycode} long pressed for {press_duration:.2f} seconds"

                            self.publisher_.publish(button_press_msg)
                            self.get_logger().info(button_press_msg.data)
                            # Remove the keycode from the dictionary after calculating the duration
                            del button_press_times[key_event.keycode]


def main(args=None):
    set_color("green")
    rclpy.init(args=args)
    front_panel_button_controller = FrontPanelButtonController()
    rclpy.spin(front_panel_button_controller)
    front_panel_button_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# /dev/input/event1 is front panel button device
