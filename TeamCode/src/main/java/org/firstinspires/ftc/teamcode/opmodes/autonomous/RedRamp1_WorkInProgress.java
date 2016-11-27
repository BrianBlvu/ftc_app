package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.HardwareK9bot;

/**
 * Created by Thomas on 11/19/2016.
 */


/**
 * Created by Thomas on 11/19/2016.
 */
@Autonomous(name="RedRamp1_WorkInProgress", group="Autonomous")
public class RedRamp1_WorkInProgress extends LinearOpMode {
    {
    }
    /* Declare OpMode members. */
    HardwareK9bot robot           = new HardwareK9bot();              // Use a K9'shardware
    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
    final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo

    @Override
    public void runOpMode() {
        double left;
        double right;

        long moveForward1Millisecond = 1000;
        long turnRightMilliseconds = 1000;
        long moveForward2Milliseconds = 2000;
        long moveForaward7Milliseconds = 700;
        double motorSpeed = 0.5;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
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
            robot.frontLeftMotor.setPower(left);
            robot.frontRightMotor.setPower(right);
            try {
                robot.waitForTick(moveForward1Millisecond);
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
            robot.frontLeftMotor.setPower(left);
            robot.frontRightMotor.setPower(right);
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
            robot.frontLeftMotor.setPower(left);
            robot.frontRightMotor.setPower(right);
            try {
                robot.waitForTick(moveForward2Milliseconds);
                telemetry.addData("Say", "Forward");
                telemetry.update();
            }
            catch(Exception e)
            {
                telemetry.addData("Exception", e.getMessage());
            }
            //turn left
            left = -motorSpeed;
            right = motorSpeed;
            robot.frontLeftMotor.setPower(left);
            robot.frontRightMotor.setPower(right);
            try {
                robot.waitForTick(turnRightMilliseconds);
                telemetry.addData("Say", "Left");
                telemetry.update();
            }
            catch(Exception e)
            {
                telemetry.addData("Exception", e.getMessage());
            }
            // move forward

            left = motorSpeed;
            right = motorSpeed;
            robot.frontLeftMotor.setPower(left);
            robot.frontRightMotor.setPower(right);
            try {
                robot.waitForTick(moveForward1Millisecond);
                telemetry.addData("Say", "Forward");
                telemetry.update();
            }
            catch(Exception e)
            {
                telemetry.addData("Exception", e.getMessage());
            }
            // turn right
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            break;

        }
    }
}
