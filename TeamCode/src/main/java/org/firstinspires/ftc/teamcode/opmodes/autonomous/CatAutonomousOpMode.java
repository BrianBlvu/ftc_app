package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ChainDriveBot1;
import org.firstinspires.ftc.teamcode.lib.Util;

import static org.firstinspires.ftc.teamcode.lib.Util.MENU_DELAY;
import static org.firstinspires.ftc.teamcode.lib.Util.printControlerStatusToTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.READY_TO_START;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.SELECT_MISSION_OPTION_START_DELAY;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.SELECT_MISSION_OPTION_TEAM_COLOR;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.CatAutonomousOpMode.State.STOPPED;

public abstract class CatAutonomousOpMode extends LinearOpMode {
    protected final double TOTAL_RUN_TIME_SECONDS = 30.0;
    protected final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    protected final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    protected StateBasedBeaconPusher.State currentState = STOPPED;
    protected ElapsedTime runtime = new ElapsedTime();
    protected Util.Color alliance = Util.Color.BLUE; // This is just a default. It's replaced in menu selection.
    ChainDriveBot1 robot = new ChainDriveBot1(telemetry);
    private StateBasedBeaconPusher.State previousState = STOPPED;
    private ElapsedTime stateTimer = new ElapsedTime();
    protected int startDelayInSeconds = 0;
    protected Util.StartPosition startPosition = Util.StartPosition.SQUARE_VILLE;
    private String currentMessage = null;

    protected void telemetryMenu() {
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
                                startPosition = Util.StartPosition.RAMP;
                                break;
                            case RAMP:
                                startPosition = Util.StartPosition.SQUARE_VILLE;
                                break;
                            case SQUARE_VILLE:
                                startPosition = Util.StartPosition.MIDDLE;
                                break;
                        }
                    } else if(gamepad1.a) {
                        changeState(SELECT_MISSION_OPTION_TEAM_COLOR);
                    }
                    break;
                case SELECT_MISSION_OPTION_TEAM_COLOR:
                    telemetry.addData("Blue_Team Or Red_Team? ", String.format("(%s)", String.valueOf(alliance)));
                    telemetry.update();
                    if (gamepad1.dpad_down) {
                        alliance = (alliance == Util.Color.RED) ? Util.Color.BLUE : Util.Color.RED;
                    } else if(gamepad1.a) {
                        changeState(SELECT_MISSION_OPTION_START_DELAY);
                    }
                    break;
                case SELECT_MISSION_OPTION_START_DELAY:
                    telemetry.addData("Start Delay ", String.format("(%s)", String.valueOf(startDelayInSeconds)));
                    telemetry.update();
                    if (gamepad1.dpad_down){
                        startDelayInSeconds++;
                        if (startDelayInSeconds > 25) startDelayInSeconds = 25;
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
    }

    public void printMessageToTelemetry(String message) {
        currentMessage = message;
        printStatusToTelemetry();
    }

    public void printStatusToTelemetry() {
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

    protected void stopMotors() {
        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);
    }

    protected void changeState(StateBasedBeaconPusher.State newState) {
        previousState = currentState;
        currentState = newState;
    }

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
        SELECT_MISSION_OPTION_START_DELAY,
        RUN_TO_BALL,
        WAIT_FOR_HALF_LIFE_2_EPISODE_3,
        ROLL_TO_THE_CENTER_VORTEX
    }
}
