package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.hardware.ChainDriveBot1;
import static org.firstinspires.ftc.teamcode.lib.Util.StartPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.lib.Util.StartPosition;
import static org.firstinspires.ftc.teamcode.lib.Util.Color.RED;
import static org.firstinspires.ftc.teamcode.lib.Util.Color;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.SELECT_MISSION_OPTION_START_POSITION;

@Autonomous(name="PushBallAndPark", group="Autonomous")
public class PushBallAndPark extends CatAutonomousOpMode {
    final double MOTOR_POWER = 0.5;

    ChainDriveBot1 robot           = new ChainDriveBot1(telemetry);

    @Override
    public void runOpMode() {
        double left;
        double right;

        long moveForward1Millisecond = 1000;
        long turnRightMilliseconds = 1000;
        long moveForward2Milliseconds = 2000;

        StartPosition startPosition = MIDDLE;
        Color Color = RED;
        int delayInSeconds = 5;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        changeState(SELECT_MISSION_OPTION_START_POSITION);
        telemetryMenu();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // TODO: Add delay

            telemetry.addData("Say", "IN WHILE STATEMENT");
            telemetry.update();

            //move forward
            left = MOTOR_POWER;
            right = MOTOR_POWER;
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

            left = -MOTOR_POWER;
            right = MOTOR_POWER;
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

            left = MOTOR_POWER;
            right = MOTOR_POWER;
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
            /*//turn right
            left = MOTOR_POWER;
            right = -MOTOR_POWER;
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
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

            left = MOTOR_POWER;
            right = MOTOR_POWER;
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
            }*/
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
