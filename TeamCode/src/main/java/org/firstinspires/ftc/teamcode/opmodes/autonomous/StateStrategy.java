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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ChainDriveBot1;

import static org.firstinspires.ftc.teamcode.opmodes.autonomous.StateStrategy.State.START;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.StateStrategy.State.STARTED;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.StateStrategy.State.STOP;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.StateStrategy.State.STOPPED;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the beaconPusher slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the beaconPusher servo approaches 0, the beaconPusher opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="StateStrategy", group="Autonomous")
public class StateStrategy extends LinearOpMode {
    enum State {
        STOP,
        STOPPED,
        START,
        STARTED,
        TURN,
        PRESS
    }

    enum Color {
        BLUE,
        RED,
        WHITE
    }

    ChainDriveBot1 robot = new ChainDriveBot1(telemetry);
    ColorSensor colorSensorBottom;
    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight;

    private State currentState = START;
    private Color currColor = null;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();

    private int ELAPSED_TIME_STATE_SWITCH = 5;

    private double leftSpeed = 0;
    private double rightSpeed = 0;

    private int END_TIME = 15;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        colorSensorBottom = hardwareMap.colorSensor.get("sensor_color");
        colorSensorLeft = hardwareMap.colorSensor.get("sensor_color");
        colorSensorRight = hardwareMap.colorSensor.get("sensor_color");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Ready to Start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.startTime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            sendSensorTelemetry();

            // Use gyro to turn right 45 degree so the robot is pointed at the white line of the first beacon

            // Roll forward until detecting the line, using the gyro to roll straight

            // Follow the white line until within pushing range of the button

            // Detect color until we see the right alliance color on one side

            // Push the correct side until color changes or too much time elapsed
            // if too much time elapsed, back up and try again

            // if the color changed, turn and go press the next beacon




            if (colorSensorBottom.red() > 200 && colorSensorBottom.green() > 200 && colorSensorBottom.blue() > 200)
            {
                telemetry.addData("Say", "FOUND WHITE LINE");
                telemetry.addData("Time: ", runtime.milliseconds());
                telemetry.update();
                currentState = STOP;
            }

            if (currentState == STARTED && stateTimer.seconds() > ELAPSED_TIME_STATE_SWITCH)
            {
                telemetry.addData("Say", "STOPPING");
                telemetry.addData("Time: ", runtime.milliseconds());
                telemetry.update();
                currentState = STOP;
                stateTimer.reset();
            }

            if (currentState == STOPPED && stateTimer.seconds() > ELAPSED_TIME_STATE_SWITCH)
            {
                telemetry.addData("Say", "STARTING_DELAY");
                telemetry.addData("Time: ", runtime.milliseconds());
                telemetry.update();
                currentState = START;
                stateTimer.reset();
            }
            switch(currentState) {
                case STOP:
                    stopMotors();
                    break;
                case TURN:
                    turn("left");
                    break;
                case START:
                    startMotors();
                default:
                    // do nothing
            }

            if (runtime.seconds() > END_TIME)
            {
                break;
            }
        }
    }

    private void sendSensorTelemetry() {
        telemetry.addData("Red  ", colorSensorBottom.red());
        telemetry.addData("Green", colorSensorBottom.green());
        telemetry.addData("Blue ", colorSensorBottom.blue());
        telemetry.addData("Time: ", runtime.milliseconds());
        telemetry.update();
    }

    public void stopMotors()
    {
        telemetry.addData("Say", "STOP MOTORS");
        telemetry.addData("Time: ", runtime.milliseconds());
        telemetry.update();
        stateTimer.startTime();
        leftSpeed = 0;
        rightSpeed = 0;

        robot.leftMotor.setPower(leftSpeed);
        robot.rightMotor.setPower(rightSpeed);
        currentState = STOPPED;
    }

    public void startMotors()
    {
        telemetry.addData("Say", "START MOTORS");
        telemetry.addData("Time: ", runtime.milliseconds());
        telemetry.update();
        stateTimer.startTime();
        leftSpeed = .3;
        rightSpeed = .3;

        robot.leftMotor.setPower(leftSpeed);
        robot.rightMotor.setPower(rightSpeed);
        currentState = STARTED;
    }

    public void turn(String direction)
    {

    }
}
