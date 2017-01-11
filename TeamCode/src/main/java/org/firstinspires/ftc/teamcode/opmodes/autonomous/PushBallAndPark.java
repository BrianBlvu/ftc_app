package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.ChainDriveBot1;

/**
 * Created by Thomas on 11/19/2016.
 */
@Disabled
@Autonomous(name="AutonomousMode1Blue", group="Autonomous")
public class PushBallAndPark extends LinearOpMode {
    {
    }
    /* Declare OpMode members. */
    ChainDriveBot1 robot           = new ChainDriveBot1(telemetry);              // Use a K9'shardware

    @Override
    public void runOpMode() {
        double left;
        double right;

        long moveForward1Millisecond = 1000;
        long turnRightMilliseconds = 1000;
        long moveForward2Milliseconds = 2000;
        double motorSpeed = 0.5;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Evil Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Say", "IN WHILE STATEMENT");
            telemetry.update();

            //move forward
            left = motorSpeed;
            right = motorSpeed;
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
            try {
                robot.waitForTick(moveForward2Milliseconds);
                telemetry.addData("Say", "Forward");
                telemetry.update();
            }
            catch(Exception e)
            {
                telemetry.addData("Exception", e.getMessage());
            }
            // turn left

            left = -motorSpeed;
            right = motorSpeed;
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
            try {
                robot.waitForTick(turnRightMilliseconds);
                telemetry.addData("Say", "Right");
                telemetry.update();
            }
            catch(Exception e)
            {
                telemetry.addData("Exception", e.getMessage());
            }
            // move forward

            left = motorSpeed;
            right = motorSpeed;
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
            try {
                robot.waitForTick(moveForward2Milliseconds);
                telemetry.addData("Say", "Forward");
                telemetry.update();
            }
            catch(Exception e)
            {
                telemetry.addData("Exception", e.getMessage());
            }

            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            break;

        }
    }
}
