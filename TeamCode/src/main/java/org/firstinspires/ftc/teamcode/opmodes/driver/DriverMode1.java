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
import org.firstinspires.ftc.teamcode.lib.Util;

import static org.firstinspires.ftc.teamcode.lib.Util.printControllerStatusToTelemetry;

/**
 * This OpMode uses the ChainDriveBot1 class to define the devices on the robot.
 * All device access is managed through the ChainDriveBot1 class. (See this class for device names)
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for the ChainDrive1 bot
 * It moves the beacon pusher with the Gampad X and B buttons respectively.
 */

@TeleOp(name = "DriverMode1", group = "driver")
public class DriverMode1 extends LinearOpMode {

    private ChainDriveBot1 robot = new ChainDriveBot1(telemetry);
    private double beaconPusherPosition = ChainDriveBot1.BEACON_PUSHER_HOME; // Servo safe position
    private String currentMessage = null;

    @Override
    public void runOpMode() {
        double leftMotorPower = 0;
        double rightMotorPower = 0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        try {
            robot.init(hardwareMap);
        } catch (Exception e) {
            printMessageToTelemetry("Failed to Initialize Robot: " + e.getMessage());
        }

        // We'll toggle line following mode with the Y button
        // wasYAlreadyPressed and isYPressed represent the previous and current state of the y button.
        boolean wasYAlreadyPressed = false;
        boolean isYPressed = false;

        boolean isFollowingLine = false;

        printMessageToTelemetry("DriverMode1: Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        printMessageToTelemetry("DriverMode1: Starting");

        try {
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                printStatusToTelemetry();

                leftMotorPower = gamepad1.left_stick_y;
                rightMotorPower = gamepad1.right_stick_y;
                isYPressed = gamepad1.y;

                // check for button state transitions.
                if (isYPressed && !wasYAlreadyPressed) {
                    // button is transitioning to a pressed state. So Toggle line following
                    isFollowingLine = !isFollowingLine;

                }

                wasYAlreadyPressed = isYPressed;

                if (isFollowingLine) {
                    Util.driveAlongLineEdge(robot, null);

                    continue;
                }

                // If either trigger is pulled more than a little bit, cut the robot's speed 5x
                if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                    leftMotorPower = leftMotorPower / 5;
                    rightMotorPower = rightMotorPower / 5;
                }

                if (robot.leftMotor != null) {
                    robot.leftMotor.setPower(leftMotorPower);
                } else {
                    printMessageToTelemetry("left motor not installed");
                }
                if (robot.rightMotor != null) {
                    robot.rightMotor.setPower(rightMotorPower);
                } else {
                    printMessageToTelemetry("right motor not installed");
                }

                // Use gamepad X & B to open and close the beaconPusher. Use dpad_up to move slowly forward
                if (gamepad1.x) {
                    beaconPusherPosition += ChainDriveBot1.BEACON_PUSHER_SPEED;
                } else if (gamepad1.b) {
                    beaconPusherPosition -= ChainDriveBot1.BEACON_PUSHER_SPEED;
                }

                if (gamepad1.left_stick_button) {
                    while (gamepad1.left_stick_button) {
                        robot.rightMotor.setPower(-0.15);
                        robot.leftMotor.setPower(-0.15);
                    }
                }

                if (gamepad1.right_stick_button) {
                    while (gamepad1.right_stick_button) {
                        robot.rightMotor.setPower(0.15);
                        robot.leftMotor.setPower(0.15);
                    }
                }

                beaconPusherPosition = Range.clip(beaconPusherPosition,
                        ChainDriveBot1.BEACON_PUSHER_MIN_RANGE,
                        ChainDriveBot1.BEACON_PUSHER_MAX_RANGE);

                if (null != robot.beaconPusher) {
                    robot.beaconPusher.setPosition(beaconPusherPosition);
                } else {
                    printMessageToTelemetry("No beaconPusher installed");
                }

                // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
                robot.waitForTick(40);
            }
        } catch (Exception e) {
            printMessageToTelemetry("Exception hit: " + e.getMessage());
        }
    }

    public void printMessageToTelemetry(String message) {
        currentMessage = message;
        printStatusToTelemetry();
    }

    public void printStatusToTelemetry() {
        if (currentMessage != null) {
            telemetry.addData("Message", currentMessage);
        }
        robot.printRobotStatusToTelemetry(this);
        printControllerStatusToTelemetry(telemetry, gamepad1);
        telemetry.update();
    }
}