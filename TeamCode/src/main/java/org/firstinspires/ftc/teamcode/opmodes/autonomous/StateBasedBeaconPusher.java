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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.ChainDriveBot1;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.StateBasedBeaconPusher.State.*;
import static org.firstinspires.ftc.teamcode.lib.Util.Button.LEFT;
import static org.firstinspires.ftc.teamcode.lib.Util.Button.RIGHT;
import static org.firstinspires.ftc.teamcode.lib.Util.Color.RED;
import static org.firstinspires.ftc.teamcode.lib.Util.Color.BLUE;
import static org.firstinspires.ftc.teamcode.lib.Util.*;
import java.text.DecimalFormat;

@Autonomous(name="StateBasedBeaconPusher", group="Autonomous")
public class StateBasedBeaconPusher extends LinearOpMode {

    enum State {
        STOPPED,
        READY_TO_START,
        WAITING_FOR_CALIBRATION,
        STARTING_DELAY,
        START_TURNING_TO_FIRST_LINE,
        TURNING_TO_FIRST_LINE,
        MOVING_TO_FIRST_LINE,
        FOLLOWING_LINE,
        READING_BEACON_COLORS,
        PUSHING_BEACON_BUTTON,
        BACKING_UP,
        PLACEHOLDER_FOR_SECOND_BEACON_STATES,
        SELECT_MISSION_OPTION_START_POSITION,
        SELECT_MISSION_OPTION_TEAM_COLOR,
        SELECT_MISSION_OPTION_START_DELAY
    }

    ChainDriveBot1 robot = new ChainDriveBot1(telemetry);

    private State previousState = STOPPED;
    private State currentState = STOPPED;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();

    /* This is the port on the Core Device Interface Module       */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */

    private boolean pushedFirstBeacon = false;
    private boolean pushedSecondBeacon = false;
    private double beaconPusherPosition = ChainDriveBot1.BEACON_PUSHER_HOME; // Servo safe position

    private final double TOTAL_RUN_TIME_SECONDS = 30.0;

    private navXPIDController yawPIDController;
    private final double TARGET_ANGLE_DEGREES = 90.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private final int DEVICE_TIMEOUT_MS = 500;
    private final DecimalFormat decimalFormat = new DecimalFormat("#.##");

    private final int BEACON_DISTANCE_THRESHOLD = 10; //In centimeters TODO: Calibrate beacon distance threshold with testing
    private final int LINE_FOLLOWING_THRESHOLD_VALUE = 5;

    private Color alliance = Color.BLUE; // This is just a default. It's replaced in menu selection.
    private int startDelayInSeconds = 0;
    private StartPosition startPosition = StartPosition.SQUARE_VILLE;
    private String currentMessage = null;

    @Override
    public void runOpMode() {
        Color current_team;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.colorDown.enableLed(true);
        robot.colorFrontLeft.enableLed(false);
        robot.colorFrontRight.enableLed(false);
        changeState(SELECT_MISSION_OPTION_START_POSITION);

        // Get the menu options for start position, alliance, and start delay
        while(currentState != READY_TO_START){
            switch (currentState)
            {
                case SELECT_MISSION_OPTION_START_POSITION:
                    telemetry.addData("Start Position? ", String.format("(%s)", String.valueOf(startPosition)));
                    telemetry.update();
                    if (gamepad1.dpad_down) {
                        switch(startPosition)
                        {
                            case MIDDLE:
                                startPosition = StartPosition.RAMP;
                                break;
                            case RAMP:
                                startPosition = StartPosition.SQUARE_VILLE;
                                break;
                            case SQUARE_VILLE:
                                startPosition = StartPosition.MIDDLE;
                                break;
                        };
                    } else if(gamepad1.a) {
                        changeState(SELECT_MISSION_OPTION_TEAM_COLOR);
                    }
                    break;
                case SELECT_MISSION_OPTION_TEAM_COLOR:
                    telemetry.addData("Blue_Team Or Red_Team? ", String.format("(%s)", String.valueOf(alliance)));
                    telemetry.update();
                    if (gamepad1.dpad_down) {
                        alliance = (alliance == Color.RED) ? Color.BLUE : Color.RED;
                    } else if(gamepad1.a) {
                        changeState(SELECT_MISSION_OPTION_START_DELAY);
                    }
                    break;
                case SELECT_MISSION_OPTION_START_DELAY:
                    telemetry.addData("Start Delay ", String.format("(%s)", String.valueOf(startDelayInSeconds)));
                    telemetry.update();
                    if (gamepad1.dpad_down){
                        startDelayInSeconds++;
                        if (startDelayInSeconds > 20) startDelayInSeconds = 20;
                    } else if (gamepad1.dpad_up) {
                        startDelayInSeconds--;
                        if (startDelayInSeconds < 0) startDelayInSeconds = 0;
                    } else if(gamepad1.a) {
                        changeState(READY_TO_START);
                    }
                    break;
            }
            sleep(MENU_DELAY);
        }

        printMessageToTelemetry("Options Selected. Ready to Start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.startTime();
        Button buttonToPush = LEFT;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            printStatusToTelemetry();
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
                    sleep(startDelayInSeconds *1000) ;
                    changeState(START_TURNING_TO_FIRST_LINE);
                    break;
                case START_TURNING_TO_FIRST_LINE:
                    setTargetAngle(45);
                    changeState(TURNING_TO_FIRST_LINE);
                    break;
                case TURNING_TO_FIRST_LINE:
                    //TODO: Fix turning code so it doesn't turn forever
                    //turnUntilAtTargetAngle(); // something odd about this function -- perpetual movement
                    changeState(MOVING_TO_FIRST_LINE);
                    break;
                case MOVING_TO_FIRST_LINE:
                    if (isOnBeaconLineEdge()) {
                        changeState(FOLLOWING_LINE);
                    }
                    break;
                case FOLLOWING_LINE:
                    if (isCloseEnoughToBeacon()) {
                        changeState(READING_BEACON_COLORS);
                    } else {
                        driveAlongLineEdge();
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
        }
    }

    private void printMessageToTelemetry(String message) {
        currentMessage = message;
        printStatusToTelemetry();
    }
    private void printStatusToTelemetry() {
        if (currentMessage != null) {
            telemetry.addData("Message", currentMessage);
        }
        telemetry.addData("", "Aliance: " + alliance + " StartPosition: " + startPosition
                + " Delay " + startDelayInSeconds + "s");
        telemetry.addData("State", currentState + " PreviousState: " + previousState);
        robot.printRobotStatusToTelemetry(this);
        printControlerStatusToTelemetry(telemetry, gamepad1);
        telemetry.update();
    }

    private void driveAlongLineEdge() {
        // TODO: Implement https://ftc-tricks.com/proportional-line-follower/
        double color;
        color = robot.colorDown.red();
        if (color >= LINE_FOLLOWING_THRESHOLD_VALUE) {
            robot.leftMotor.setPower(-0.25);
            robot.rightMotor.setPower(0.1);
            printMessageToTelemetry("Adjusting speed of left motor: " + robot.leftMotor.getPower());
        } else if (color < LINE_FOLLOWING_THRESHOLD_VALUE) {
            robot.rightMotor.setPower(-0.15);
            robot.leftMotor.setPower(0);
            printMessageToTelemetry("Adjusting speed of right motor: " + robot.rightMotor.getPower());
        }
    }

    private boolean isCloseEnoughToBeacon() {
       int currentDistance = 255;
        while (currentDistance > BEACON_DISTANCE_THRESHOLD ) {

         // TODO: Read sensor and place in currentDistance
            currentDistance = getSensorDistance();
        }




        return robot.beaconDistance.getLightDetected() > BEACON_DISTANCE_THRESHOLD;
    }

    private int getSensorDistance(){
        int opticalDistanceCm;
        int ultrasonicDistanceCm;

        I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
        final int RANGE1_REG_START = 0x04; //Register to start reading
        final int RANGE1_READ_LENGTH = 2; //Number of byte to read

        I2cDevice RANGE1;
        I2cDeviceSynch RANGE1Reader;
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();


        byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        ultrasonicDistanceCm = range1Cache[0] & 0xFF;
        opticalDistanceCm = range1Cache[1] & 0xFF;

        printMessageToTelemetry("Range optical = " + opticalDistanceCm);
        printMessageToTelemetry("Range ultrasonic = " + ultrasonicDistanceCm);
       // telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
       // telemetry.addData("ODS", range1Cache[1] & 0xFF);
       // telemetry.addData("Status", "Run Time: " + runtime.toString());
       // telemetry.update();

        if (opticalDistanceCm > ultrasonicDistanceCm)
            return ultrasonicDistanceCm;
        else
            return  opticalDistanceCm;

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
        if (beaconPusherPosition <= ChainDriveBot1.BEACON_PUSHER_LEFT_PUSHING_POSITION
            || beaconPusherPosition >= ChainDriveBot1.BEACON_PUSHER_RIGHT_PUSHING_POSITION) {
            return true;
        } else {
            return false;
        }
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

    private void stopMotors() {
        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);
    }

    private boolean isOnBeaconLineEdge() {
        return robot.colorDown.red() > LINE_FOLLOWING_THRESHOLD_VALUE;
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

    private void changeState(State newState) {
        previousState = currentState;
        currentState = newState;
    }
}
