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

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Boolean;
import java.text.*;

import org.firstinspires.ftc.teamcode.hardware.ChainDriveBot1;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.StateBasedBeaconPusher.State.*;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.StateBasedBeaconPusher.Button.LEFT;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.StateBasedBeaconPusher.Button.RIGHT;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.StateBasedBeaconPusher.Color.RED;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.StateBasedBeaconPusher.Color.BLUE;
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
        SELECT_MISSION_OPTION_BEACONS
    }

    enum Color {
        RED,
        BLUE
    }

    enum StartPosition {
        Ramp,
        Middle,
        SquareVille
    }

    enum Button {
        LEFT,
        RIGHT
    }

    ChainDriveBot1 robot = new ChainDriveBot1(telemetry);

    private State currentState = STOPPED;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();

    /* This is the port on the Core Device Interface Module        */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */
    // TODO: Move NavX setup code to ChainDriveBot1 class
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navXDevice;
    private navXPIDController yawPIDController;
    private boolean calibration_complete = false;
    private boolean pushedFirstBeacon = false;
    private boolean pushedSecondBeacon = false;
    private double beaconPusherPosition = ChainDriveBot1.BEACON_PUSHER_HOME; // Servo safe position

    private final double TOTAL_RUN_TIME_SECONDS = 30.0;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private final double TARGET_ANGLE_DEGREES = 90.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private final int DEVICE_TIMEOUT_MS = 500;
    private final DecimalFormat decimalFormat = new DecimalFormat("#.##");

    private final double BEACON_DISTANCE_THRESHOLD = 0.5; // TODO: Calibrate beacon distance threshold with testing
    private final int LINE_FOLLOWING_THRESHOLD_VALUE = 5;

    // TODO: feed in the actual alliance name
    private Color alliance = Color.BLUE;
    private boolean getBeacons = true;
    private StartPosition startPosition = StartPosition.SquareVille;

    @Override
    public void runOpMode() {
        Color current_team;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        initializeNavigationController();
        robot.colorDown.enableLed(true);
        robot.colorFrontLeft.enableLed(false);
        robot.colorFrontRight.enableLed(false);
        Boolean INITIALIZE_MISSION_OPTIONS = true;
        changeState(SELECT_MISSION_OPTION_START_POSITION);
        //start initialization phase
        while(INITIALIZE_MISSION_OPTIONS){
            switch (currentState)
            {
                case SELECT_MISSION_OPTION_START_POSITION:
                    telemetry.addData("Say", String.format("Start Position? : (%s)", String.valueOf(startPosition)));
                    telemetry.update();
                    if (gamepad1.dpad_down){
                        sleep(250);
                        switch(startPosition)
                        {
                            case Middle:
                                startPosition = StartPosition.Ramp;
                                break;
                            case Ramp:
                                startPosition = StartPosition.SquareVille;
                                break;
                            case SquareVille:
                                startPosition = StartPosition.Middle;
                                break;

                        };
                    }
                    if(gamepad1.a)
                    {
                        sleep(250);
                        changeState(SELECT_MISSION_OPTION_TEAM_COLOR);
                    }
                    break;
                case SELECT_MISSION_OPTION_TEAM_COLOR:
                    telemetry.addData("Say", String.format("Blue_Team Or Red_Team? : (%s)", String.valueOf(alliance)));
                    telemetry.update();
                    if (gamepad1.dpad_down){
                        sleep(750);
                        alliance = (alliance == Color.RED) ? Color.BLUE : Color.RED;
                    }
                    if(gamepad1.a)
                    {
                        sleep(250);
                        changeState(SELECT_MISSION_OPTION_BEACONS);
                    }
                    break;
                case SELECT_MISSION_OPTION_BEACONS:
                    telemetry.addData("Say", String.format("Push Beacons? : (%s)", String.valueOf(getBeacons)));
                    telemetry.update();
                    if (gamepad1.dpad_down){
                        getBeacons = !getBeacons;
                    }
                    if(gamepad1.a)
                    {
                        sleep(250);
                        INITIALIZE_MISSION_OPTIONS = false;
                    }
                    break;
            }

        }
        //end initionalization phase
        changeState(READY_TO_START);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Ready to Start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.startTime();
        Button buttonToPush = LEFT;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.printStatusToTelemetry(this);
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
                    calibrateNavigationBoard();
                    changeState(STARTING_DELAY);
                    break;
                case STARTING_DELAY:
                    // TODO: Add delay code
                    changeState(START_TURNING_TO_FIRST_LINE);
                    break;
                case START_TURNING_TO_FIRST_LINE:
                    setTargetAngle(45);
                    changeState(TURNING_TO_FIRST_LINE);
                    break;
                case TURNING_TO_FIRST_LINE:
                    turnUntilAtTargetAngle();
                    changeState(MOVING_TO_FIRST_LINE);
                    break;
                case MOVING_TO_FIRST_LINE:
                    if (isOnBeaconLineEdge()) {
                        changeState(FOLLOWING_LINE);
                    }
                    break;
                case FOLLOWING_LINE:
                    if (isCloseEnoughToBeacon()) {
                        stopMotors();
                        changeState(READING_BEACON_COLORS);
                    } else {
                        driveAlongLineEdge();
                    }
                    break;
                case READING_BEACON_COLORS:
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

    private void driveAlongLineEdge() {
        // TODO: Implement https://ftc-tricks.com/proportional-line-follower/
        double color;
        color = robot.colorDown.red();
        if (color > LINE_FOLLOWING_THRESHOLD_VALUE) {
            robot.leftMotor.setPower(0.5);
            robot.rightMotor.setPower(0.0);
        } else if (color < LINE_FOLLOWING_THRESHOLD_VALUE) {
            robot.rightMotor.setPower(0.5);
            robot.leftMotor.setPower(0.0);
        }
    }

    private boolean isCloseEnoughToBeacon() {
        return robot.beaconDistance.getLightDetected() > BEACON_DISTANCE_THRESHOLD;
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
                        telemetry.addData("PIDOutput", decimalFormat.format(0.00));
                    } else {
                        double output = yawPIDResult.getOutput();
                        robot.leftMotor.setPower(output);
                        robot.rightMotor.setPower(-output);
                        telemetry.addData("PIDOutput", decimalFormat.format(output) + ", " +
                                decimalFormat.format(-output));
                    }
                } else {
                /* A timeout occurred */
                    Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
                telemetry.addData("Yaw", decimalFormat.format(navXDevice.getYaw()));
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        } finally {
            navXDevice.close();
            telemetry.addData("NavX", "Interrupted");
        }
    }

    private void calibrateNavigationBoard() {
        while (!calibration_complete) {
                        /* navX-Micro Calibration completes automatically ~15 seconds after it is
                        powered on, as long as the device is still.  To handle the case where the
                        navX-Micro has not been able to calibrate successfully, hold off using
                        the navX-Micro Yaw value until calibration is complete.
                         */
            calibration_complete = !navXDevice.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }
        navXDevice.zeroYaw();
    }

    private void setTargetAngle(double targetAngleInDegress) {
        yawPIDController.setSetpoint(targetAngleInDegress);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
    }

    private void initializeNavigationController(){


//        navXDevice = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("nav"),
//                NAVX_DIM_I2C_PORT,
//                AHRS.DeviceDataType.kProcessedData,
//                NAVX_DEVICE_UPDATE_RATE_HZ);

        DeviceInterfaceModule dim = hardwareMap.deviceInterfaceModule.iterator().next();
        telemetry.addData("DIM Name", dim.getDeviceName());
        navXDevice = AHRS.getInstance(dim,
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        /* If possible, use encoders when driving, as it results in more */
        /* predictable drive system response.                           */
        //leftMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //rightMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController(navXDevice,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
    }

    private void changeState(State newState) {
        telemetry.addData("Changing State", "Old: " + currentState + "New: " + newState);
        currentState = newState;
    }
}
