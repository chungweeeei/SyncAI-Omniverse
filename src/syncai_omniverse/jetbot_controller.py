"""Keyboard controller for Jetbot using DifferentialController."""

import numpy as np
import carb
import carb.input
import omni.appwindow

from isaacsim.robot.wheeled_robots.controllers import DifferentialController


class KeyboardJetbotController:
    def __init__(self, wheel_radius: float, wheel_base: float,
                 max_linear_speed: float, max_angular_speed: float):
        self._max_linear = max_linear_speed
        self._max_angular = max_angular_speed
        self._linear_speed = 0.0
        self._angular_speed = 0.0

        self._diff_controller = DifferentialController(
            name="jetbot_diff_controller",
            wheel_radius=wheel_radius,
            wheel_base=wheel_base,
        )

        self._input_iface = None
        self._keyboard = None
        self._sub = None

    def setup_keyboard(self):
        self._input_iface = carb.input.acquire_input_interface()
        app_window = omni.appwindow.get_default_app_window()
        self._keyboard = app_window.get_keyboard()
        self._sub = self._input_iface.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event
        )

    def get_action(self):
        return self._diff_controller.forward(
            command=np.array([self._linear_speed, self._angular_speed])
        )

    def cleanup(self):
        if self._sub is not None and self._input_iface is not None:
            self._input_iface.unsubscribe_to_keyboard_events(self._sub)
            self._sub = None

    # ------------------------------------------------------------------ #
    #  Keyboard callback
    # ------------------------------------------------------------------ #
    def _on_keyboard_event(self, event, *args, **kwargs):
        KI = carb.input.KeyboardInput
        pressed = event.type == carb.input.KeyboardEventType.KEY_PRESS
        released = event.type == carb.input.KeyboardEventType.KEY_RELEASE

        if event.input in (KI.W, KI.UP):
            self._linear_speed = self._max_linear if pressed else 0.0
        elif event.input in (KI.S, KI.DOWN):
            self._linear_speed = -self._max_linear if pressed else 0.0
        elif event.input in (KI.A, KI.LEFT):
            self._angular_speed = self._max_angular if pressed else 0.0
        elif event.input in (KI.D, KI.RIGHT):
            self._angular_speed = -self._max_angular if pressed else 0.0

        return True
