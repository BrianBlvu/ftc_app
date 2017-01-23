package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.kauailabs.navx.ftc.IDataArrivalSubscriber;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.ChainDriveBot1;

import static org.firstinspires.ftc.teamcode.lib.Util.Color;
import static org.firstinspires.ftc.teamcode.lib.Util.Color.RED;
import static org.firstinspires.ftc.teamcode.lib.Util.StartPosition;
import static org.firstinspires.ftc.teamcode.lib.Util.StartPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.DONE;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.ROLL_TO_THE_CENTER_VORTEX;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.RUN_TO_BALL;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.SELECT_MISSION_OPTION_START_POSITION;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.STARTING_DELAY;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.WAITING_FOR_CALIBRATION;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.WAIT_FOR_HALF_LIFE_2_EPISODE_3;

@Autonomous(name="PushBallAndParkSuperSimple", group="Autonomous")
public class PushBallAndParkSuperSimple extends CatAutonomousOpMode implements IDataArrivalSubscriber {
    final double MOTOR_POWER = 0.5;

    ChainDriveBot1 robot           = new ChainDriveBot1(telemetry);

    double last_world_linear_accel_x;
    double last_world_linear_accel_y;

    boolean haveWeHitTheBall = false;

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
        printMessageToTelemetry("Options Selected. Ready to Start");
        sleep(1000);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            printStatusToTelemetry();


        }

    private boolean isTouchingBall() {
        return false;
    }

    @Override
    public void untimestampedDataReceived(long l, Object o) {

    }

    @Override
    public void timestampedDataReceived(long curr_system_timestamp, long curr_sensor_timestamp, Object o) {
        boolean collisionDetected = false;

        double curr_world_linear_accel_x = robot.navXDevice.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = robot.navXDevice.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;

        if ( ( Math.abs(currentJerkX) > robot.COLLISION_THRESHOLD_DELTA_G) ||
                ( Math.abs(currentJerkY) > robot.COLLISION_THRESHOLD_DELTA_G) ) {
            collisionDetected = true;
        }

        haveWeHitTheBall=true;
    }

    @Override
    public void yawReset() {

    }
}

