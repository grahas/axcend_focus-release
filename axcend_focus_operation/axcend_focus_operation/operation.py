import numpy as np
from scipy.integrate import trapezoid
from scipy.interpolate import interp1d
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse

from axcend_focus_custom_interfaces.msg import (
    InstrumentState, 
    CartridgeOvenControl,
    PumpStatus
)
from axcend_focus_custom_interfaces.action import (
    ValveRotate,
    PumpPositioning,
    PumpPressurizeSystem,
    Method,
    CartridgeOvenSet,
    Pump,
)
from axcend_focus_custom_interfaces.srv import (
    CartridgeMemoryReadWrite,
)
from axcend_focus_device_config import device_config

class OperationNode(Node):
    """The operation node for the Axcend Focus instrument."""

    def __init__(self):
        super().__init__('operation_node')

        # Create a publisher for the instrument state
        self.instrument_state_publisher = self.create_publisher(
            InstrumentState, "instrument_state", 1
        )
        self.instrument_state_cache_msg = InstrumentState()

        # Create a service to handle the method
        self.method_action = ActionServer(
            self,
            Method,
            "method_action",
            execute_callback=self.callback_method_handler,
            goal_callback=self.goal_callback,
        )

        # Create action clients for the pump, pressurize system, oven set, and rotate valves
        self.pump_positioning_action = ActionClient(self, PumpPositioning, 'pump_positioning')
        self.pump_action = ActionClient(self, Pump, 'pump')
        self.pump_pressurize_system_action = ActionClient(self, PumpPressurizeSystem, 'pump_pressurize_system')
        self.oven_set_action = ActionClient(self, CartridgeOvenSet, 'cartridge_oven_set')
        self.valve_rotate = ActionClient(self, ValveRotate, 'valve_rotate')

        # Create a service call to the cartridge memory service
        self.cartridge_memory_service_client = self.create_client(CartridgeMemoryReadWrite, 'cartridge_memory')
        self.cartridge_memory_service_cache = CartridgeMemoryReadWrite.Response()

        # Wait for all nodes to be ready
        self.get_logger().info('Waiting for all nodes to be ready...')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    def callback_method_handler(self, goal_handle):
        """Handle the method request."""
        def splice_point(goal, injection_time):
            # Create an interpolation function
            interp_func = interp1d(goal.gradient_x, goal.gradient_y, kind='linear', fill_value="extrapolate")

            # Calculate the new y-value at the injection time
            new_y = interp_func(injection_time)

            # Find the position to insert the new point
            pos = np.searchsorted(goal.gradient_x, injection_time)

            # Insert the new point into the gradient arrays
            goal.gradient_x = np.insert(goal.gradient_x, pos, injection_time)
            goal.gradient_y = np.insert(goal.gradient_y, pos, new_y)

        # Get the goal
        goal = goal_handle.request

        # Check the goal to make sure it is valid
        # Check the flow rate is non zero
        if goal.flow_rate == 0:
            goal_handle.abort()
            return

        # Create the feedback message
        feedback_msg = Method.Feedback()
        feedback_msg.current_step = Method.Feedback.SETUP
        goal_handle.publish_feedback(feedback_msg)

        # Check if the cartridge has an oven, and if the oven is to be used for the method
        # Make a service call to the cartridge memory service
        request = CartridgeMemoryReadWrite.Request()
        request.command = "read"
        self.cartridge_memory_service_cache = self.cartridge_memory_service_client.call(request)

        # If the cartridge has an oven then check if the oven is at the target temperature
        if (
            self.cartridge_memory_service_cache.cartridge_memory.has_oven
            and goal.oven_control.oven_state == goal.OVEN_ON
        ):
            # Set the insturmnet state to oven heating
            self.instrument_state_cache_msg.state = InstrumentState.TEMPERATURE_CONTROL
            self.instrument_state_publisher.publish(self.instrument_state_cache_msg)

            # Call the oven set action
            oven_goal_msg = CartridgeOvenSet.Goal()
            oven_goal_msg.oven_control.oven_state = goal.OVEN_ON
            oven_goal_msg.oven_control.oven_set_point = goal.oven_set_point
            result = self.oven_set_action.send_goal(oven_goal_msg) # Block until at target temperature

            # Check if we are at the target
            if result.is_at_target_temperature is False:
                # We are not at the target temperature, abort the method
                goal_handle.abort()
                return

        # Update the current step
        feedback_msg.current_step = Method.Feedback.POSITIONING
        goal_handle.publish_feedback(feedback_msg)

        # Create the pump positioning goal
        pump_goal_msg = PumpPositioning.Goal()

        # Position the pump to the required volume
        if goal.control_mode == Method.Goal.PRESSURE_MODE: # If the pump is in pressure mode then the target starting volume is 95% full
            pump_goal_msg.percentage[PumpStatus.PUMP_A] = device_config.get_config()["pump"]["pressure_mode_fill_percentage"]
            pump_goal_msg.percentage[PumpStatus.PUMP_B] = device_config.get_config()["pump"]["pressure_mode_fill_percentage"]
        elif goal.control_mode == Method.Goal.FLOW_MODE:
            # Calculate the volume
            gradient_max_value = 100

            # Calculate the area of gradient part B, unit is percent seconds
            area_b = trapezoid(goal.gradient_y, goal.gradient_x)

            # Calculate the total area of the gradient and remove part B to get part A, 100 is 100%
            area_a = goal.gradient_x[-1] * gradient_max_value - area_b
            
            # Calculate the volume required for the gradient
            pump_goal_msg.volume[PumpStatus.PUMP_A] = (
                area_a * goal.flow_rate * (1/60) # Add the gradient volume, percent seconds * uL/min * (1 min / 60 seconds) = uL
                + int(device_config.system_parameter_read_key("pumpSyringeAVolume")) * (device_config.get_config()["pump"]["disposition_mode_safety_margin_percentage"] / 100)
                + (goal.equilibration_time) * goal.flow_rate * (1/60) * (gradient_max_value - goal.gradient_y[0]) # Add the equilibration volume
            )
            pump_goal_msg.volume[PumpStatus.PUMP_B] = (
                area_b * goal.flow_rate * (1/60) # Add the gradient volume
                + int(device_config.system_parameter_read_key("pumpSyringeBVolume")) * (device_config.get_config()["pump"]["disposition_mode_safety_margin_percentage"] / 100)
            )
        else:
            # Invalid control mode
            goal_handle.abort()
            return
        
        # Call the pump positioning action
        result = self.pump_positioning_action.send_goal(pump_goal_msg) # Block until at target volume

        # Bring the system up to the operating pressure
        feedback_msg.current_step = Method.Feedback.PRE_PRESSURIZING
        goal_handle.publish_feedback(feedback_msg)

        # Create the pressurize system goal
        pressurize_goal_msg = PumpPressurizeSystem.Goal()
        pressurize_goal_msg.pressure = goal.operating_pressure

        # Call the pressurize system action
        result = self.pump_pressurize_system_action.send_goal(pressurize_goal_msg) # Block until at target pressure

        # Rotate the valves to be inline with the column
        valve_goal_msg = ValveRotate.Goal() # Create the valve goal
        valve_goal_msg.valve_position = [ValveRotate.Goal.VALVE_SOLVENT_AB, ValveRotate.Goal.VALVE_INJECT_LOAD] # Inline with the column
        result = self.valve_rotate.send_goal(valve_goal_msg)

        # Pump at the starting point of the gradient
        pump_goal_msg = Pump.Goal() # Create the pump goal
        pump_goal_msg.time = 90 # From Paul, best to just operate in pressure mode for 90 seconds to push everything out
        pump_goal_msg.start_ratio = goal.gradient_y[0] # First point on the gradient
        pump_goal_msg.end_ratio = goal.gradient_y[0] # First point on the gradient
        pump_goal_msg.target_pressure = goal.operating_pressure
        result = self.pump_action.send_goal(pump_goal_msg) # Blocking call

        # Bringing the system up to pressure introduces a large slug of A solvent
        # We must push out the slug of A solvent before we can start the method
        feedback_msg.current_step = Method.Feedback.EQUILIBRATION
        goal_handle.publish_feedback(feedback_msg)

        # Pump at the starting point of the gradient
        pump_goal_msg = Pump.Goal() # Create the pump goal
        pump_goal_msg.time = goal.equilibration_time # Equilibration time
        pump_goal_msg.start_ratio = goal.gradient_y[0] # First point on the gradient
        pump_goal_msg.end_ratio = goal.gradient_y[0] # First point on the gradient
        pump_goal_msg.flow_rate = goal.flow_rate # Flow rate
        result = self.pump_action.send_goal(pump_goal_msg) # Blocking call

        # Handle the injection
        feedback_msg.current_step = Method.Feedback.INJECTION
        goal_handle.publish_feedback(feedback_msg)

        # Rotate the valves to the injection position
        if goal_handle.injection_type != Method.Goal.NO_INJECTION:
            valve_goal_msg = ValveRotate.Goal() # Create the valve goal
            valve_goal_msg.valve_position = [ValveRotate.Goal.VALVE_SOLVENT_AB, ValveRotate.Goal.VALVE_INJECT_INJECT] # Inline with the column
            result = self.valve_rotate.send_goal(valve_goal_msg)

        if goal_handle.injection_type == Method.Goal.TIMED_INJECTION:
            # Check if the injection is time or volume based
            if goal_handle.injection_volume != 0 and goal_handle.injection_time == 0: # Volume based injection
                # Calculate the time to inject in seconds where the flow_rate is in ul/min and volume is in nl
                goal_handle.injection_time = (goal_handle.injection_volume * 1e-3 / goal.flow_rate) * 60
            
            # Splice the gradient at the injection time
            splice_point(goal, goal_handle.injection_time)
            
            # Create the pump goal
            pump_goal_msg = Pump.Goal()
            pump_goal_msg.time = goal_handle.injection_time
            pump_goal_msg.start_ratio = goal.gradient_y[0]
            pump_goal_msg.end_ratio = goal.gradient_y[np.searchsorted(goal.gradient_x, goal_handle.injection_time)]
            pump_goal_msg.flow_rate = goal.flow_rate
            result = self.pump_action.send_goal(pump_goal_msg) # Blocking call

            # Close the injection valve after the injection is complete
            valve_goal_msg = ValveRotate.Goal()
            valve_goal_msg.valve_position = [ValveRotate.Goal.VALVE_SOLVENT_AB, ValveRotate.Goal.VALVE_INJECT_LOAD] # Inline with the column
            result = self.valve_rotate.send_goal(valve_goal_msg)

        # Handle running the gradient
        feedback_msg.current_step = Method.Feedback.RUNNING
        goal_handle.publish_feedback(feedback_msg)

        # Check if timed injection was used, if it is then start on the post splice point
        if goal_handle.injection_type == Method.Goal.TIMED_INJECTION:
            start_index = np.searchsorted(goal.gradient_x, goal_handle.injection_time)
        else:
            start_index = 0

        # Run the gradient
        for i in range(start_index, len(goal.gradient_x) - 1):
            # Create the pump goal
            pump_goal_msg = Pump.Goal()
            pump_goal_msg.time = goal.gradient_x[i + 1] - goal.gradient_x[i]
            pump_goal_msg.start_ratio = goal.gradient_y[i]
            pump_goal_msg.end_ratio = goal.gradient_y[i + 1]
            pump_goal_msg.flow_rate = goal.flow_rate
            result = self.pump_action.send_goal(pump_goal_msg) # Blocking call

        # Handle the post pressure release
        feedback_msg.current_step = Method.Feedback.TEARDOWN
        goal_handle.publish_feedback(feedback_msg)

        # Rotate the valves to the fill and load position
        valve_goal_msg = ValveRotate.Goal() # Create the valve goal
        valve_goal_msg.valve_position = [ValveRotate.Goal.VALVE_SOLVENT_FILL, ValveRotate.Goal.VALVE_INJECT_LOAD]
        result = self.valve_rotate.send_goal(valve_goal_msg)

        # Mark the method as completed
        feedback_msg.current_step = Method.Feedback.COMPLETED
        goal_handle.publish_feedback(feedback_msg)

        # Make the results
        result = Method.Result()
        result.success = True

        # Return a response
        goal_handle.succeed()

        # Set the instrument state to idle
        self.instrument_state_cache_msg.state = InstrumentState.IDLE
        self.instrument_state_publisher.publish(self.instrument_state_cache_msg)

        return result
    
def main():
    print('Hi from axcend_focus_operation.')


if __name__ == '__main__':
    main()
