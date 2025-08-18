#!/usr/bin/env python3
"""
Automatic Figure-8 Pattern Controller

This program automatically starts a figure-8 walking pattern as soon as the robot
calibration is complete (when "Calibration state: 0" is printed).

The figure-8 pattern consists of:
1. Move forward (4s)
2. Move right (4s) 
3. Move forward (4s)
4. Turn left and move forward (4s)
5. Turn left and move forward (4s)
6. Move left (4s)
7. Move forward (4s)
8. Turn right and move forward (4s)
9. Turn right and move forward (4s)
10. Loop back to step 1

Usage:
    export ROBOT_TYPE=PF_TRON1A
    python figure8_auto_controller.py [robot_ip]
"""

import os
import sys
import time
import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes
import controllers as controllers
from threading import Thread, Event
import signal


class ComprehensiveEvaluationController:
    def __init__(self, robot, robot_type, robot_ip):
        self.robot = robot
        self.robot_type = robot_type
        self.robot_ip = robot_ip
        self.controller = None
        self.running = False
        self.stop_event = Event()
        self.calibration_complete = False
        self.pattern_started = False
        
        # Comprehensive evaluation pattern to test all aspects of policy performance
        self.comprehensive_pattern = [
            # Basic locomotion tests
            {'linear_x': 0.3, 'linear_y': 0.0, 'angular_z': 0.0, 'duration': 3.0, 'description': 'Forward walk (slow)'},
            {'linear_x': 1.0, 'linear_y': 0.0, 'angular_z': 0.0, 'duration': 2.0, 'description': 'Forward walk (fast)'},
            {'linear_x': -0.3, 'linear_y': 0.0, 'angular_z': 0.0, 'duration': 3.0, 'description': 'Backward walk'},
            
            # Lateral movement tests
            {'linear_x': 0.0, 'linear_y': 0.4, 'angular_z': 0.0, 'duration': 3.0, 'description': 'Left strafe'},
            {'linear_x': 0.0, 'linear_y': -0.4, 'angular_z': 0.0, 'duration': 3.0, 'description': 'Right strafe'},
            
            # Rotation tests
            {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.8, 'duration': 2.5, 'description': 'Rotate left (in place)'},
            {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': -0.8, 'duration': 2.5, 'description': 'Rotate right (in place)'},
            
            # Combined movements (agility tests)
            {'linear_x': 0.4, 'linear_y': 0.3, 'angular_z': 0.0, 'duration': 2.0, 'description': 'Diagonal forward-left'},
            {'linear_x': 0.4, 'linear_y': -0.3, 'angular_z': 0.0, 'duration': 2.0, 'description': 'Diagonal forward-right'},
            {'linear_x': 0.3, 'linear_y': 0.0, 'angular_z': 0.5, 'duration': 3.0, 'description': 'Arc left (walk + turn)'},
            {'linear_x': 0.3, 'linear_y': 0.0, 'angular_z': -0.5, 'duration': 3.0, 'description': 'Arc right (walk + turn)'},
            
            # Quick direction changes (stability tests)
            {'linear_x': 0.5, 'linear_y': 0.0, 'angular_z': 0.0, 'duration': 1.0, 'description': 'Quick forward'},
            {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0, 'duration': 0.5, 'description': 'Stop'},
            {'linear_x': -0.4, 'linear_y': 0.0, 'angular_z': 0.0, 'duration': 1.5, 'description': 'Quick backward'},
            {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0, 'duration': 0.5, 'description': 'Stop'},
            
            # Complex maneuvers
            {'linear_x': 0.3, 'linear_y': 0.2, 'angular_z': 0.3, 'duration': 2.0, 'description': 'Complex: forward-left-turn'},
            {'linear_x': 0.2, 'linear_y': -0.3, 'angular_z': -0.4, 'duration': 2.0, 'description': 'Complex: slow-right-turn'},
            
            # Endurance test
            {'linear_x': 0.4, 'linear_y': 0.0, 'angular_z': 0.0, 'duration': 5.0, 'description': 'Sustained forward walk'},
            
            # Recovery test (stop and stabilize)
            {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0, 'duration': 2.0, 'description': 'Final stabilization'},
        ]
        
        
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        print("\nShutting down evaluation controller...")
        self.stop_event.set()
        self.running = False
        if self.controller:
            self.controller.start_controller = False
        sys.exit(0)
        
    def custom_diagnostic_callback(self, diagnostic_value: datatypes.DiagnosticValue):
        """Custom diagnostic callback to detect calibration completion"""
        if diagnostic_value.name == "calibration":
            print(f"Calibration state: {diagnostic_value.code}")
            if diagnostic_value.code == 0 and not self.calibration_complete:
                self.calibration_complete = True
                print("🎯 Calibration complete! Starting comprehensive evaluation pattern in 1 seconds...")
                # Start pattern after a short delay
                Thread(target=self.delayed_pattern_start, daemon=True).start()
                
    def delayed_pattern_start(self):
        """Start the pattern after a short delay"""
        time.sleep(1.0)
        if not self.pattern_started and self.controller:
            self.pattern_started = True
            time.sleep(2.0)  # Wait for controller to start
            print("🚀 Starting comprehensive evaluation pattern!")
            Thread(target=self.run_comprehensive_pattern, daemon=True).start()
        
    def start_controller(self):
        """Start the robot controller in a separate thread"""
        model_dir = f'{os.path.dirname(os.path.abspath(__file__))}/controllers/model'
        start_controller = False  # Don't auto-start, wait for calibration
        
        if self.robot_type.startswith("PF"):
            self.controller = controllers.PointfootController(model_dir, self.robot, self.robot_type, start_controller)
        elif self.robot_type.startswith("WF"):
            self.controller = controllers.WheelfootController(model_dir, self.robot, self.robot_type, start_controller)
        elif self.robot_type.startswith("SF"):
            self.controller = controllers.SolefootController(model_dir, self.robot, self.robot_type, start_controller)
        else:
            raise ValueError(f"Unknown robot type: {self.robot_type}")
            
        # Override the diagnostic callback to detect calibration completion
        original_callback = self.controller.robot_diagnostic_callback_partial
        
        def combined_callback(diagnostic_value):
            # Call original callback
            original_callback(diagnostic_value)
            # Call our custom callback
            self.custom_diagnostic_callback(diagnostic_value)
            
        self.controller.robot_diagnostic_callback_partial = combined_callback
        self.robot.subscribeDiagnosticValue(combined_callback)
            
        # Start controller in background thread
        controller_thread = Thread(target=self.controller.run, daemon=True)
        controller_thread.start()

        # Start the controller
        self.simulate_button_press("start")
        
        print("Controller started. Waiting for calibration to complete...")
        
    def apply_joystick_command(self, linear_x, linear_y, angular_z):
        """Apply joystick command directly to the controller"""
        if self.controller:
            # Clamp values to [-1, 1] range
            linear_x = max(-1.0, min(1.0, linear_x))
            linear_y = max(-1.0, min(1.0, linear_y))
            angular_z = max(-1.0, min(1.0, angular_z))
            
            # Apply commands with offsets (same logic as in sensor_joy_callback)
            self.controller.commands[0] = linear_x
            self.controller.commands[1] = linear_y
            self.controller.commands[2] = angular_z
            self.controller.commands += self.controller.command_offsets
            
    def simulate_button_press(self, button_combination):
        """Simulate button press combinations"""
        if button_combination == "start" and self.controller:
            print("🎮 Simulating L1 + Y button press - Starting controller")
            self.controller.start_controller = True
        elif button_combination == "stop" and self.controller:
            print("🎮 Simulating L1 + X button press - Stopping controller")
            self.controller.start_controller = False
            
    def run_comprehensive_pattern(self):
        """Run the comprehensive evaluation pattern once"""
        total_steps = len(self.comprehensive_pattern)
        
        print(f"🔥 Starting evaluation sequence ({total_steps} steps)")
        
        for step_index, step in enumerate(self.comprehensive_pattern):
            if not self.running or self.stop_event.is_set():
                break
                
            linear_x = step['linear_x']
            linear_y = step['linear_y']
            angular_z = step['angular_z']
            duration = step['duration']
            description = step['description']
            
            print(f"📍 Step {step_index + 1}/{total_steps}: {description}")
            print(f"   Commands: linear_x={linear_x:.1f}, linear_y={linear_y:.1f}, angular_z={angular_z:.1f}")
            print(f"   Duration: {duration}s")
            
            # Execute the step for the specified duration
            start_time = time.time()
            while time.time() - start_time < duration and not self.stop_event.is_set():
                self.apply_joystick_command(linear_x, linear_y, angular_z)
                time.sleep(0.1)  # Update at 10Hz
                
        print("✅ Comprehensive evaluation pattern completed!")
        
    def run(self):
        """Main run method"""
        signal.signal(signal.SIGINT, self.signal_handler)
        self.running = True
        
        print("🤖 Comprehensive Evaluation Controller Started")
        print(f"Robot Type: {self.robot_type}")
        print(f"Robot IP: {self.robot_ip}")
        print("Waiting for calibration to complete...")
        print("(The comprehensive evaluation pattern will start automatically when 'Calibration state: 0' is detected)")
        
        # Start the robot controller
        self.start_controller()
        
        try:
            # Keep the main thread alive
            while self.running and not self.stop_event.is_set():
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
            
        # Stop the controller
        if self.controller:
            self.simulate_button_press("stop")
            
        print("\n🛑 Comprehensive Evaluation Controller stopped.")


def main():
    # Get robot type from environment variable
    robot_type = os.getenv("ROBOT_TYPE")
    if not robot_type:
        print("Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.")
        print("Example: export ROBOT_TYPE=PF_TRON1A")
        sys.exit(1)
    
    # Create robot instance
    robot = Robot(RobotType.PointFoot)
    
    # Get robot IP
    robot_ip = "127.0.0.1"
    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]
    
    # Initialize robot
    if not robot.init(robot_ip):
        print("Failed to initialize robot")
        sys.exit(1)
    
    # Create and run controller
    controller = ComprehensiveEvaluationController(robot, robot_type, robot_ip)
    controller.run()


if __name__ == "__main__":
    main()