#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging

from ..utils import TeleopEvents


class InputController:
    """Base class for input controllers that generate motion deltas."""

    def __init__(
            self,
            translational_sens=0.02, #meters
            rotational_sens=2, #degrees 
            x_step_size=None,
            y_step_size=None,
            z_step_size=None, 
            roll_step_size=None, 
            pitch_step_size=None, 
            yaw_step_size=None
            ):
        """
        Initialize the controller.

        Args:
            x_step_size: Base movement step size in meters
            y_step_size: Base movement step size in meters
            z_step_size: Base movement step size in meters
        """
        if translational_sens is None:
            self.x_step_size = x_step_size
            self.y_step_size = y_step_size
            self.z_step_size = z_step_size
        else:
            self.x_step_size = translational_sens
            self.y_step_size = translational_sens
            self.z_step_size = translational_sens
        if rotational_sens is None:
            self.roll_step_size = roll_step_size
            self.pitch_step_size = pitch_step_size
            self.yaw_step_size = yaw_step_size
        else:
            self.roll_step_size = rotational_sens
            self.pitch_step_size = rotational_sens
            self.yaw_step_size = rotational_sens
        self.running = True
        self.episode_end_status = None  # None, "success", or "failure"
        self.intervention_flag = False
        self.gripper_command = False #open

    def start(self):
        """Start the controller and initialize resources."""
        pass

    def stop(self):
        """Stop the controller and release resources."""
        pass

    def get_deltas(self):
        """Get the current movement deltas (dx, dy, dz) in meters."""
        return 0.0, 0.0, 0.0

    def should_quit(self):
        """Return True if the user has requested to quit."""
        return not self.running

    def update(self):
        """Update controller state - call this once per frame."""
        pass

    def __enter__(self):
        """Support for use in 'with' statements."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Ensure resources are released when exiting 'with' block."""
        self.stop()

    def get_episode_end_status(self):
        """
        Get the current episode end status.

        Returns:
            None if episode should continue, "success" or "failure" otherwise
        """
        status = self.episode_end_status
        self.episode_end_status = None  # Reset after reading
        return status

    def should_intervene(self):
        """Return True if intervention flag was set."""
        return self.intervention_flag

class GamepadController(InputController):
    """Generate motion deltas from gamepad input."""

    def __init__(
            self,
            translational_sens=0.02, #meters
            rotational_sens=2, #degrees 
            x_step_size=None,
            y_step_size=None,
            z_step_size=None, 
            roll_step_size=None, 
            pitch_step_size=None, 
            yaw_step_size=None,
            deadzone=0.1
            ):
        super().__init__(
            translational_sens=translational_sens,
            rotational_sens=rotational_sens,
            x_step_size=x_step_size,
            y_step_size=y_step_size,
            z_step_size=z_step_size,
            roll_step_size=roll_step_size,
            pitch_step_size=pitch_step_size,
            yaw_step_size=yaw_step_size
        )
        self.deadzone = deadzone
        self.joystick = None
        self.intervention_flag = False
        self.yaw_roll_toggle = False

    def start(self):
        """Initialize pygame and the gamepad."""
        import pygame

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            logging.error("No gamepad detected. Please connect a gamepad and try again.")
            self.running = False
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        logging.info(f"Initialized gamepad: {self.joystick.get_name()}")

        print('init done')
        # print("Gamepad controls:")
        # print("  Left analog stick: Move in X-Y plane")
        # print("  Right analog stick (vertical): Move in Z axis")
        # print("  B/Circle button: Exit")
        # print("  Y/Triangle button: End episode with SUCCESS")
        # print("  A/Cross button: End episode with FAILURE")
        # print("  X/Square button: Rerecord episode")

    def stop(self):
        """Clean up pygame resources."""
        import pygame

        if pygame.joystick.get_init():
            if self.joystick:
                self.joystick.quit()
            pygame.joystick.quit()
        pygame.quit()

    def update(self):
        """Process pygame events to get fresh gamepad readings."""
        import pygame

        for event in pygame.event.get():
            # print()
            # for i in range(15): 
            #     try: 
            #         print(f'{i}: {self.joystick.get_button(i)}')
            #     except:
            #         pass
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0: # X
                    self.episode_end_status = TeleopEvents.SUCCESS
                # A button (1) for failure
                elif event.button == 1: # circle
                    self.episode_end_status = TeleopEvents.FAILURE
                # X button (0) for rerecord
                elif event.button == 2: # triangle
                    self.episode_end_status = TeleopEvents.RERECORD_EPISODE

                # RB button (6) for closing gripper
                elif event.button == 5: #rb
                    self.gripper_command = True

                # LT button (7) for opening gripper
                elif event.button == 4: #lb
                    self.gripper_command = False
                
                elif event.button == 12: #right stick press
                    self.yaw_roll_toggle = not self.yaw_roll_toggle

            # Reset episode status on button release
            elif event.type == pygame.JOYBUTTONUP:
                if event.button in [0, 1, 2]:
                    self.episode_end_status = None

            # Check for square button (typically button 2) for intervention flag
            if self.joystick.get_button(3):
                self.intervention_flag = True
            else:
                self.intervention_flag = False

    def get_deltas(self):
        """Get the current movement deltas from gamepad state."""
        import pygame

        try:
            # print()
            # for i in range(6): print(self.joystick.get_axis(i))
            # Read joystick axes
            # Left stick X and Y (typically axes 0 and 1)
            y_input = self.joystick.get_axis(0)
            z_input = -self.joystick.get_axis(1)

            x_input = (self.joystick.get_axis(5) - self.joystick.get_axis(2))/2

            pitch_input = -self.joystick.get_axis(4)
            if self.yaw_roll_toggle:
                yaw_input = self.joystick.get_axis(3)
                roll_input = 0.0
            else:
                roll_input = self.joystick.get_axis(3)
                yaw_input = 0.0

            # Apply deadzone to avoid drift
            x_input = 0 if abs(x_input) < self.deadzone else x_input
            y_input = 0 if abs(y_input) < self.deadzone else y_input
            z_input = 0 if abs(z_input) < self.deadzone else z_input
            roll_input = 0 if abs(roll_input) < self.deadzone else roll_input
            pitch_input = 0 if abs(pitch_input) < self.deadzone else pitch_input
            yaw_input = 0 if abs(yaw_input) < self.deadzone else yaw_input  

            # Calculate deltas (note: may need to invert axes depending on controller)
            delta_x = x_input * self.x_step_size  # Forward/backward
            delta_y = y_input * self.y_step_size  # Left/right
            delta_z = z_input * self.z_step_size  # Up/down

            delta_roll = roll_input * self.roll_step_size  # Roll
            delta_pitch = pitch_input * self.pitch_step_size  # Pitch
            delta_yaw = yaw_input * self.yaw_step_size  # Yaw

            # if delta_roll != 0 or delta_pitch != 0 or delta_yaw != 0:
            #     print(delta_roll,delta_pitch, delta_yaw)

            return [delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw]

        except pygame.error:
            logging.error("Error reading gamepad. Is it still connected?")
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0