/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.opmodes.driver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.ChainDriveBot1;

/**
 * This OpMode uses the ChainDriveBot1 class to define the devices on the robot.
 * All device access is managed through the ChainDriveBot1 class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the ChainDrive1 bot
 * It moves the beacon pusher with the Gampad X and B buttons respectively.
 */

@TeleOp(name="DriverMode1", group="driver")
public class DriverMode1 extends LinearOpMode {

    private ChainDriveBot1 robot = new ChainDriveBot1();
    private double beaconPusherPosition = ChainDriveBot1.BEACON_PUSHER_HOME; // Servo safe position

    @Override
    public void runOpMode() {
        double leftMotorPower;
        double rightMotorPower;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        try {
            robot.init(hardwareMap);
        } catch (Exception e)
        {
            telemetry.addData("Failed to Initialize Robot: ",  e.getMessage());
        }

        // We'll toggle the color sensor LEDs on and off with the Y button
        // wasYAlreadyPressed and isYPressed represent the previous and current state of the y button.
        boolean wasYAlreadyPressed = false;
        boolean isYPressed = false;

        boolean isLedOn = true;

        // Set the color sensor LEDs on in the beginning
        robot.colorDown.enableLed(true);
        robot.colorFrontLeft.enableLed(true);
        robot.colorFrontRight.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("DriverMode1", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("DriverMode1", "Starting");
        telemetry.update();

        try{
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                leftMotorPower = gamepad1.left_stick_y;
                rightMotorPower = gamepad1.right_stick_y;
                isYPressed = gamepad1.y;

                // check for button state transitions.
                if (isYPressed && !wasYAlreadyPressed) {
                    // button is transitioning to a pressed state. So Toggle LED
                    isLedOn = !isLedOn;
                    robot.colorDown.enableLed(isLedOn);
                    robot.colorFrontLeft.enableLed(isLedOn);
                    robot.colorFrontRight.enableLed(isLedOn);
                }

                wasYAlreadyPressed = isYPressed;

                // If either trigger is pulled more than a little bit, cut the robot's speed 10x
                if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                    leftMotorPower = leftMotorPower / 10;
                    rightMotorPower = rightMotorPower / 10;
                }

                if (robot.leftMotor != null) {
                    robot.leftMotor.setPower(leftMotorPower);
                } else {
                    telemetry.addData("left motor not installed", "");
                }
                if (robot.rightMotor != null) {
                    robot.rightMotor.setPower(rightMotorPower);
                } else {
                    telemetry.addData("right motor not installed", "");
                }

                // Use gamepad X & B to open and close the beaconPusher
                if (gamepad1.x)
                    beaconPusherPosition += ChainDriveBot1.BEACON_PUSHER_SPEED;
                else if (gamepad1.b)
                    beaconPusherPosition -= ChainDriveBot1.BEACON_PUSHER_SPEED;

                beaconPusherPosition = Range.clip(beaconPusherPosition,
                        ChainDriveBot1.BEACON_PUSHER_MIN_RANGE,
                        ChainDriveBot1.BEACON_PUSHER_MAX_RANGE);

                if (null != robot.beaconPusher) {
                    robot.beaconPusher.setPosition(beaconPusherPosition);
                } else {
                    telemetry.addData("No beaconPusher installed", "");
                }

                printStatusToTelemetry();

                // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
                robot.waitForTick(40);
            }
        } catch (Exception e) {
            telemetry.addData("Exception hit: ", e.getMessage());
        }
    }

    private void printStatusToTelemetry() {
        // show the values we're currently feeding the motors
        telemetry.addData("Controls", "left: %.2f, right: %.2f, beaconPusher: %.2f",
                robot.leftMotor.getPower(), robot.rightMotor.getPower(), beaconPusherPosition);

        telemetry.addData("Color RGB", "down:%d/%d/%d, left:%d/%d/%d, right%d/%d/%d",
                robot.colorDown.red(), robot.colorDown.green(), robot.colorDown.blue(),
                robot.colorFrontLeft.red(), robot.colorFrontLeft.green(), robot.colorFrontLeft.blue(),
                robot.colorFrontRight.red(), robot.colorFrontRight.green(), robot.colorFrontRight.blue());
        telemetry.addData("Distance to beacon", robot.beaconDistance);

        // show the state of the current controls
        telemetry.addData("Controller1", "lsx:%.2f lsy:%.2f lsb:%b",
                gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.left_stick_button);
        telemetry.addData("Controller1", "rsx:%.2f rsy:%.2f rsb:%b",
                gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.right_stick_button);
        telemetry.addData("Controller1", "lt:%.2f rt:%.2f lb:%.2b rb:%.2b",
                gamepad1.left_trigger, gamepad1.right_trigger, gamepad1.left_bumper,
                gamepad1.right_bumper);
        telemetry.addData("Controller1", "dpad: l:%b r:%b u:%b d:%b", gamepad1.dpad_left,
                gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down);
        telemetry.addData("Controller1", "x:%b y:%b a:%b b:%b", gamepad1.x, gamepad1.y,
                gamepad1.a, gamepad1.b);
        telemetry.update();
    }
}
