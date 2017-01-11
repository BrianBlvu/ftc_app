package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Util {
    public static final int MENU_DELAY = 250;

    public static void printControlerStatusToTelemetry(Telemetry telemetry, Gamepad gamepad) {
        // show the state of the current controls
        telemetry.addData("Controller1", "lsx:%.2f lsy:%.2f lsb:%b",
                gamepad.left_stick_x, gamepad.left_stick_y, gamepad.left_stick_button);
        telemetry.addData("Controller1", "rsx:%.2f rsy:%.2f rsb:%b",
                gamepad.right_stick_x, gamepad.right_stick_y, gamepad.right_stick_button);
        telemetry.addData("Controller1", "lt:%.2f rt:%.2f lb:%.2b rb:%.2b",
                gamepad.left_trigger, gamepad.right_trigger, gamepad.left_bumper,
                gamepad.right_bumper);
        telemetry.addData("Controller1", "dpad: l:%b r:%b u:%b d:%b", gamepad.dpad_left,
                gamepad.dpad_right, gamepad.dpad_up, gamepad.dpad_down);
        telemetry.addData("Controller1", "x:%b y:%b a:%b b:%b", gamepad.x, gamepad.y,
                gamepad.a, gamepad.b);
    }

    public enum Color {
        RED,
        BLUE
    }

    public enum StartPosition {
        RAMP,
        MIDDLE,
        SQUARE_VILLE
    }

    public enum Button {
        LEFT,
        RIGHT
    }
}
