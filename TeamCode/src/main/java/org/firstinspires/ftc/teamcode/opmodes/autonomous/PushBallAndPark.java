package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.hardware.ChainDriveBot1;
import static org.firstinspires.ftc.teamcode.lib.Util.StartPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.lib.Util.StartPosition;
import static org.firstinspires.ftc.teamcode.lib.Util.Color.RED;
import static org.firstinspires.ftc.teamcode.lib.Util.Color;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.ROLL_TO_THE_CENTER_VORTEX;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.SELECT_MISSION_OPTION_START_POSITION;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.STARTING_DELAY;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.START_TURNING_TO_FIRST_LINE;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.WAITING_FOR_CALIBRATION;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.WAIT_FOR_HALF_LIFE_2_EPISODE_3;

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
            printStatusToTelemetry();
            // Here's our strategy:
            // Use gyro to turn right 45 degree so the robot is pointed at the white line of the first beacon
            // Roll forward until detecting the line, using the gyro to roll straight
            // Follow the white line until within pushing range of the button
            // Detect color until we see the right alliance color on one side
            // Push the correct side until color changes or too much time elapsed
            // if too much time elapsed, back up and try again
            // if the color changed, turn and go press the next beacon

            switch (currentState) {
                case READY_TO_START:
                    changeState(WAITING_FOR_CALIBRATION);
                    break;
                case WAITING_FOR_CALIBRATION:
                    robot.calibrateNavigationBoard();
                    changeState(STARTING_DELAY);
                    break;
                case STARTING_DELAY:
                    sleep(startDelayInSeconds * 1000);
                    changeState(START_TURNING_TO_FIRST_LINE);
                    break;
                case RUN_TO_BALL:
                    if (isTouchingBall()) {
                        changeState(WAIT_FOR_HALF_LIFE_2_EPISODE_3);
                    } else {
                        robot.leftMotor.setPower(MOTOR_POWER);
                        robot.rightMotor.setPower(MOTOR_POWER);
                    }
                    break;
                case WAIT_FOR_HALF_LIFE_2_EPISODE_3:
                    sleep(2000);
                    changeState(ROLL_TO_THE_CENTER_VORTEX);
                    break;
                }
            }
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);

        }

    private boolean isTouchingBall() {
        return false;
    }
}

