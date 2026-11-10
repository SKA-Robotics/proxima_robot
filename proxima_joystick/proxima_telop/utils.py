def max_abs(a, b):
    if abs(a) > abs(b):
        return a
    else:
        return b


def trig_to_axis(trig):
    return - (trig - 1) / 2


JOYSTICK_DATA = {
    "axes": {
        "left_stick_horizontal": 0,
        "left_stick_vertical": 1,
        "left_trigger": 2,
        "right_stick_horizontal": 3,
        "right_stick_vertical": 4,
        "right_trigger": 5,
        "dpad_horizontal": 6,
        "dpad_vertical": 7,
    },
    "buttons": {
        "A_button": 0,
        "B_button": 1,
        "X_button": 2,
        "Y_button": 3,
        "left_bumper": 4,
        "right_bumper": 5,
        "back_button": 6,
        "start_button": 7,
        "power_button": 8,
        "left_stick_button": 9,
        "right_stick_button": 10,
    }
}


class JoystickTranslator:

    def __init__(self):
        self.AXES_ID: dict = JOYSTICK_DATA["axes"]
        self.BUTTONS_ID: dict = JOYSTICK_DATA["buttons"]

    def translate(self, data):
        inputs = dict((name, data["buttons"][id])
                      for name, id in self.BUTTONS_ID.items())
        inputs.update(
            dict(
                (name, data["axes"][id]) for name, id in self.AXES_ID.items()
                if name not in ("left_trigger", "right_trigger",
                                "dpad_horizontal", "dpad_vertical")))

        # Triggers rest at +1 and go to -1 when pulled.
        inputs["left_trigger"] = trig_to_axis(
            data["axes"][self.AXES_ID["left_trigger"]])
        inputs["right_trigger"] = trig_to_axis(
            data["axes"][self.AXES_ID["right_trigger"]])
        hat_x = data["axes"][self.AXES_ID["dpad_horizontal"]]
        hat_y = data["axes"][self.AXES_ID["dpad_vertical"]]
        inputs["left_cross"] = int(hat_x < -0.5)
        inputs["right_cross"] = int(hat_x > 0.5)
        inputs["up_cross"] = int(hat_y < -0.5)
        inputs["down_cross"] = int(hat_y > 0.5)

        return inputs


class Debouncing:
    def __init__(self, threshold_low=0.45, threshold_high=0.55):
        self.state = False
        self.threshold_low = threshold_low
        self.threshold_high = threshold_high
        self.is_rising_edge = False
        self.is_falling_edge = False

    def update(self, value):
        if self.state == False and value > self.threshold_high:
            self.state = True
            self.is_rising_edge = True
            self.is_falling_edge = False
        elif self.state == True and value < self.threshold_low:
            self.state = False
            self.is_rising_edge = False
            self.is_falling_edge = True
        else:
            self.is_rising_edge = False
            self.is_falling_edge = False
