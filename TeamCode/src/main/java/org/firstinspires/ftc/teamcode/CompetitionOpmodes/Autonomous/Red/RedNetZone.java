package org.firstinspires.ftc.teamcode.CompetitionOpmodes.Autonomous.Red;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class RedNetZone extends LinearOpMode {
    Robot robot;
    MultipleTelemetry telemetry;
    StateMachine blueNetZoneAuto;
    StateMachine collapseMachine;
    StateMachine transferMachine;
    StateMachine intakeMachine;

    public static Pose startPose = (new Pose(4.745, 88.250, Math.toRadians(180))).convertToFTC().rotate180();
    public static Pose rungPoint = (new Pose(29.891, 78.761, Math.toRadians(180))).convertToFTC().rotate180();
    public static Pose firstSample = (new Pose(28.468, 121.226, Math.toRadians(20))).convertToFTC().rotate180();
    public static Pose secondSample = (new Pose(27.519, 130.715, Math.toRadians(0))).convertToFTC().rotate180();
    public static Pose thirdSample = (new Pose(25.621, 135.222, Math.toRadians(15))).convertToFTC().rotate180();
    public static Pose highBasket = (new Pose(17.081, 130.715, Math.toRadians(315))).convertToFTC().rotate180();
    public static Pose ascentZonePark = (new Pose(63.815, 94.418, Math.toRadians(90))).convertToFTC().rotate180();
    public static Point constBezierCurvePoint = new Point(4.270, 114.820, Point.CARTESIAN).rotate180();


    Pose[] samples = {firstSample, secondSample, thirdSample};

    PathChain hangSpeciman;
    PathChain pickSample;
    PathChain dropSample;
    PathChain travelToSample;
    PathChain parkAscentZone;

    int sampleCounter = 0;
    boolean startAuto = false;
    enum redNetZoneStates{
        START,
        HANG_SPECIMAN,
        TRAVEL_TO_SAMPLES,
        PICK_SAMPLE,
        DROP_SAMPLE,
        PARK,
        COLLAPSE,
        TRANSFER,
        INTAKE,
        STOP
    }

    public void runOpMode() {

    }
}
