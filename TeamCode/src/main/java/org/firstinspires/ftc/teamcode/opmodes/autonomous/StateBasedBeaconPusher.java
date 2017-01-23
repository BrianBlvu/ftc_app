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
package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Log;

import com.kauailabs.navx.ftc.navXPIDController;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.ChainDriveBot1;

import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.*;
import static org.firstinspires.ftc.teamcode.lib.Util.Button.LEFT;
import static org.firstinspires.ftc.teamcode.lib.Util.Button.RIGHT;
import static org.firstinspires.ftc.teamcode.lib.Util.Color.RED;
import static org.firstinspires.ftc.teamcode.lib.Util.Color.BLUE;
import static org.firstinspires.ftc.teamcode.lib.Util.*;
import java.text.DecimalFormat;

@Autonomous(name="StateBasedBeaconPusher", group="Autonomous")
public class StateBasedBeaconPusher extends CatAutonomousOpMode {

    /* This is the port on the Core Device Interface Module       */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */

    private boolean pushedFirstBeacon = false;
    private boolean pushedSecondBeacon = false;
    private double beaconPusherPosition = ChainDriveBot1.BEACON_PUSHER_HOME; // Servo safe position

    private navXPIDController yawPIDController;
    private final double TARGET_ANGLE_DEGREES = 90.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private final int DEVICE_TIMEOUT_MS = 500;
    private final DecimalFormat decimalFormat = new DecimalFormat("#.##");

    private final double BEACON_DISTANCE_THRESHOLD = 20; // in CM

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.colorDown.enableLed(true);
        robot.colorFrontLeft.enableLed(false);
        robot.colorFrontRight.enableLed(false);

        changeState(SELECT_MISSION_OPTION_START_POSITION);
        telemetryMenu();
        printMessageToTelemetry("Options Selected. Ready to Start");
        sleep(1000);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //printMessageToTelemetry("Started 1");
        runtime.startTime();
        //printMessageToTelemetry("Started 2");
        Button buttonToPush = LEFT;
        // run until the end of the match (driver presses STOP)
        int i = 0;
        while (opModeIsActive()) {
            printMessageToTelemetry(""+ i++);
            //printStatusToTelemetry();
            // Here's our strategy:
            // Use gyro to turn right 45 degree so the robot is pointed at the white line of the first beacon
            // Roll forward until detecting the line, using the gyro to roll straight
            // Follow the white line until within pushing range of the button
            // Detect color until we see the right alliance color on one side
            // Push the correct side until color changes or too much time elapsed
            // if too much time elapsed, back up and try again
            // if the color changed, turn and go press the next beacon

            switch (currentState)
            {
                case READY_TO_START:
                    changeState(WAITING_FOR_CALIBRATION);
                    break;
                case WAITING_FOR_CALIBRATION:
                    robot.calibrateNavigationBoard();
                    changeState(STARTING_DELAY);
                    break;
                case STARTING_DELAY:
                    sleep(startDelayInSeconds*1000);
                    changeState(START_TURNING_TO_FIRST_LINE);
                    break;
                case START_TURNING_TO_FIRST_LINE:
                    setTargetAngle(45);
                    changeState(TURNING_TO_FIRST_LINE);
                    break;
                case TURNING_TO_FIRST_LINE:
                    turnUntilAtTargetAngle(); // something odd about this function -- perpetual movement
                    changeState(MOVING_TO_FIRST_LINE);
                    break;
                case MOVING_TO_FIRST_LINE:
                    if (isOnBeaconLineEdge()) {
                        changeState(FOLLOWING_LINE);
                    } else {
                        robot.leftMotor.setPower(robot.IMPULSE_POWER);
                        robot.rightMotor.setPower(robot.IMPULSE_POWER);
                    }
                    break;
                case FOLLOWING_LINE:
                    if (isCloseEnoughToBeacon()) {
                        changeState(READING_BEACON_COLORS);
                    } else {
                        driveAlongLineEdge(robot, this);
                    }
                    break;
                case READING_BEACON_COLORS:
                    stopMotors();
                    if (getSensorColor(Button.RIGHT) == alliance) {
                        buttonToPush = Button.RIGHT;
                    } else {
                        buttonToPush = Button.LEFT;
                    }
                    changeState(PUSHING_BEACON_BUTTON);
                    break;
                case PUSHING_BEACON_BUTTON:
                    if (beaconPushedEnough()) {
                        changeState(BACKING_UP);
                    } else if (buttonToPush == Button.RIGHT) {
                        pushButton(RIGHT);
                    } else {
                        pushButton(LEFT);
                    }
                    break;
                case BACKING_UP:
                    //move backward until the ultrasonic sensor senses that we are a foot away from the beacon
                    break;
                case PLACEHOLDER_FOR_SECOND_BEACON_STATES:
                    //turn -90 degrees
                    //move forward until the robot senses the second white line
                    //turn 90 degrees
                    //case READING_BEACON_COLORS
                    //case PUSHING_BEACON_BUTTONS
                    changeState(STOPPED);
                    break;
                case STOPPED:
                    stopMotors();
            }
            robot.waitForTick(40);
        }
    }

    private boolean isCloseEnoughToBeacon() {
        return robot.beaconDistance.getDistance(DistanceUnit.CM) < BEACON_DISTANCE_THRESHOLD;
    }

    private Color getSensorColor(Button side) {
        if (side == LEFT) {
            if (robot.colorFrontLeft.red() > robot.colorFrontLeft.blue()) {
                return RED;
            } else {
                return BLUE;
            }
        } else {
            if (robot.colorFrontRight.red() > robot.colorFrontRight.blue()) {
                return RED;
            } else {
                return BLUE;
            }
        }
    }

    private boolean beaconPushedEnough() {
        return beaconPusherPosition <= ChainDriveBot1.BEACON_PUSHER_LEFT_PUSHING_POSITION
                || beaconPusherPosition >= ChainDriveBot1.BEACON_PUSHER_RIGHT_PUSHING_POSITION;
    }

    private void pushButton(Button buttonToPush) {
        if (buttonToPush == LEFT) {
            beaconPusherPosition += ChainDriveBot1.BEACON_PUSHER_SPEED;
        }
        else {
            beaconPusherPosition -= ChainDriveBot1.BEACON_PUSHER_SPEED;
        }
        beaconPusherPosition = Range.clip(beaconPusherPosition,
                ChainDriveBot1.BEACON_PUSHER_MIN_RANGE,
                ChainDriveBot1.BEACON_PUSHER_MAX_RANGE);

        if (null != robot.beaconPusher) {
            robot.beaconPusher.setPosition(beaconPusherPosition);
        } else {
            telemetry.addData("No beaconPusher installed", "");
        }
    }

    private boolean isOnBeaconLineEdge() {
        return robot.colorDown.red() > ChainDriveBot1.LINE_FOLLOWING_THRESHOLD_VALUE;
    }

    private void turnUntilAtTargetAngle() {
        yawPIDController.enable(true);

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        try {
            while ((runtime.time() < TOTAL_RUN_TIME_SECONDS) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        robot.leftMotor.setPower(0);
                        robot.rightMotor.setPower(0);
                        printMessageToTelemetry("PIDOutput: " + decimalFormat.format(0.00));
                    } else {
                        double output = yawPIDResult.getOutput();
                        robot.leftMotor.setPower(output);
                        robot.rightMotor.setPower(-output);
                        printMessageToTelemetry("PIDOutput: " + decimalFormat.format(output) + ", " +
                                decimalFormat.format(-output));
                    }
                } else {
                /* A timeout occurred */
                    printMessageToTelemetry("navXRotateOp: Yaw PID waitForNewUpdate() TIMEOUT.");
                    Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
                printMessageToTelemetry("Yaw: " + decimalFormat.format(robot.navXDevice.getYaw()));
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        } finally {
            robot.navXDevice.close();
            printMessageToTelemetry("NavX: Interrupted");
        }
    }

    private void setTargetAngle(double targetAngleInDegress) {
        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController(robot.navXDevice,
                navXPIDController.navXTimestampedDataSource.YAW);

        yawPIDController.setSetpoint(targetAngleInDegress);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
    }

}
