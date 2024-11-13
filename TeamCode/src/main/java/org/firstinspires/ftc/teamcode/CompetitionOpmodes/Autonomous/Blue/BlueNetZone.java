package org.firstinspires.ftc.teamcode.CompetitionOpmodes.Autonomous.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.Subsystems.DR4B;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.StateMachines;
import org.firstinspires.ftc.teamcode.Util.Util;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous
public class BlueNetZone extends LinearOpMode {
    Robot robot;
    StateMachine blueNetZoneAuto;
    StateMachine collapseMachine;
    StateMachine transferMachine;
    StateMachine intakeMachine;

    MultipleTelemetry multipleTelemetry;

    public static Pose startPose = (new Pose(9.015, 90, Math.toRadians(180))).convertToFTC();
    public static Pose start_hangPoint = (new Pose(28, 82, Math.toRadians(180))).convertToFTC();
    public static Pose rungPoint = (new Pose(32, 82.082, Math.toRadians(180))).convertToFTC();
    public static Pose firstSample = (new Pose(28.468, 121.226, Math.toRadians(-30))).convertToFTC();
    public static Pose secondSample = (new Pose(27.519, 130.715, Math.toRadians(0))).convertToFTC();
    public static Pose thirdSample = (new Pose(25.621, 135.222, Math.toRadians(15))).convertToFTC();
    public static Pose highBasket = (new Pose(20, 130.715, Math.toRadians(315))).convertToFTC();
    public static Pose ascentZonePark = (new Pose(63.815, 94.418, Math.toRadians(90))).convertToFTC();
    public static Point constBezierCurvePoint = new Point(5, 108.178, Point.CARTESIAN).convertToFTC();


    Pose[] samples = {secondSample, firstSample, thirdSample};

    PathChain hangSpeciman;
    PathChain startHang;
    PathChain pickSample;
    PathChain dropSample;
    PathChain travelToSample;
    PathChain parkAscentZone;

    int sampleCounter = 0;
    boolean startAuto = false;
    enum blueNetZoneStates{
        START,
        START_HANG,
        HANG_SPECIMEN,
        TRAVEL_TO_SAMPLES,
        PICK_SAMPLE,
        DROP_SAMPLE,
        PARK,
        COLLAPSE,
        COLLAPSE_TIME,
        TRANSFER,
        INTAKE,
        STOP
    }

    blueNetZoneStates previousState;

//    TODO: set dr4b and intake positions through enums

    @Override
    public void runOpMode() {
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry, startPose, true, gamepad1, gamepad2);
        collapseMachine = StateMachines.getCollapseMachine(robot, telemetry);
        transferMachine = StateMachines.getTransferMachine(robot, telemetry);
        intakeMachine = StateMachines.getIntakingMachine(robot, telemetry);
        robot.toInit();
        robot.outtake.setClaw(Outtake.CLAW_GRAB);
        robot.intakeArm.setArm(IntakeArm.ARM_TRANSFER + 0.05);


        startHang =
                new PathBuilder()
                        .addPath(
                                new BezierLine(startPose.getAsPoint(),
                                        start_hangPoint.getAsPoint()))
                        .setConstantHeadingInterpolation(startPose.getHeading())
                        .setPathEndVelocityConstraint(0.05)
                        .build();
        hangSpeciman =
                new PathBuilder()
                        .addPath(
                                new BezierLine(start_hangPoint.getAsPoint(),
                                        rungPoint.getAsPoint())
                        )
                        .setConstantHeadingInterpolation(rungPoint.getHeading())
                        .setPathEndVelocityConstraint(0.03)
                        .build();
        travelToSample =
                new PathBuilder()
                        .addPath(
                                new BezierCurve(rungPoint.getAsPoint(),
                                        constBezierCurvePoint,
                                        samples[0].getAsPoint()))
                        .setLinearHeadingInterpolation(rungPoint.getHeading(), samples[0].getHeading())
                        .build();

        parkAscentZone =
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        highBasket.getAsPoint(),
                                        ascentZonePark.getAsPoint()
                                )
                        )
                        .setLinearHeadingInterpolation(highBasket.getHeading(), ascentZonePark.getHeading())
                        .build();

        blueNetZoneAuto = new StateMachineBuilder()
                .state(blueNetZoneStates.START)
                .onEnter(() -> startAuto = true)
                .transition(() -> startAuto, blueNetZoneStates.START_HANG)
                .onExit(() -> previousState = blueNetZoneStates.START)

                .state(blueNetZoneStates.START_HANG)
                .onEnter(() -> {
                    robot.drive.drive.followPath(startHang);
                })
                .transition(() -> !robot.drive.drive.isBusy(), blueNetZoneStates.HANG_SPECIMEN)
                .onExit(() -> previousState = blueNetZoneStates.START_HANG)

                .state(blueNetZoneStates.HANG_SPECIMEN)
                .onEnter(() -> {
                    robot.dr4b.setPosition(DR4B.UPPER_SPECIMEN);
                })
                .transition(() -> Util.isCloseEnough(robot.dr4b.getAngle(), robot.dr4b.getPosition(), DR4B.acceptable), blueNetZoneStates.COLLAPSE_TIME)
                .onExit(() -> {
                    robot.drive.drive.followPath(hangSpeciman);
                    robot.outtake.setArm(Outtake.ARM_SPECIMEN);
                    robot.outtake.setWrist(Outtake.WRIST_SPECIMEN);
                    previousState = blueNetZoneStates.HANG_SPECIMEN;
                })

                .state(blueNetZoneStates.COLLAPSE_TIME)
                .transitionTimed(1.5, blueNetZoneStates.COLLAPSE)

                .state(blueNetZoneStates.COLLAPSE)
                .onEnter(() -> collapseMachine.start())
                .loop(() -> collapseMachine.update())
                .transition(() -> collapseMachine.getStateEnum() == StateMachines.CollapseStates.FINISHED && sampleCounter >= 2, blueNetZoneStates.PARK)
                .transition(() -> collapseMachine.getStateEnum() == StateMachines.CollapseStates.FINISHED && previousState == blueNetZoneStates.HANG_SPECIMEN, blueNetZoneStates.STOP) // TODO:
                .transition(() -> collapseMachine.getStateEnum() == StateMachines.CollapseStates.FINISHED && sampleCounter < 2 && previousState == blueNetZoneStates.DROP_SAMPLE, blueNetZoneStates.PICK_SAMPLE)
                .onExit(() -> {
                    collapseMachine.stop();
                    collapseMachine.reset();
                    previousState = blueNetZoneStates.COLLAPSE;
                })

                .state(blueNetZoneStates.TRANSFER)
                .onEnter(() -> transferMachine.start())
                .loop(() -> transferMachine.update())
                .transition(() -> transferMachine.getStateEnum() == StateMachines.TransferStates.FINISHED, blueNetZoneStates.DROP_SAMPLE)
                .onExit(() -> {
                    transferMachine.stop();
                    transferMachine.reset();
                    previousState = blueNetZoneStates.TRANSFER;
                })

                .state(blueNetZoneStates.INTAKE)
                .onEnter(() -> intakeMachine.start())
                .loop(() -> intakeMachine.update())
                .transition(() -> intakeMachine.getStateEnum() == StateMachines.IntakingStates.FINAL, blueNetZoneStates.TRANSFER)
                .onExit(() -> {
                    intakeMachine.stop();
                    intakeMachine.reset();
                    previousState = blueNetZoneStates.INTAKE;
                })

                .state(blueNetZoneStates.TRAVEL_TO_SAMPLES)
                .onEnter(() -> {
                    robot.drive.drive.followPath(travelToSample);
                })
                .loop(() -> {
                    Point currentPoint = robot.drive.drive.getPose().getAsPoint();
                    if (currentPoint.distanceFrom(samples[0].getAsPoint()) <= 5) {
                        robot.intakeSlides.setPosition(IntakeSlides.PARTIAL);
                        robot.intakeArm.setArm(IntakeArm.FLOAT_ARM);
                        robot.intakeArm.setSwivel(IntakeArm.swivelAngleToPos(Math.toDegrees(robot.drive.drive.getTotalHeading())));
                    }
                })
                .transition(() -> !robot.drive.drive.isBusy(), blueNetZoneStates.INTAKE)
                .onExit(() -> previousState = blueNetZoneStates.TRAVEL_TO_SAMPLES)

                .state(blueNetZoneStates.PICK_SAMPLE)
                .onEnter(() -> {
                    robot.intakeSlides.setPosition(IntakeSlides.PARTIAL);
                    pickSample = new PathBuilder()
                            .addPath(
                                    new BezierLine(rungPoint.getAsPoint(),
                                            samples[sampleCounter].getAsPoint()))
                            .setLinearHeadingInterpolation(rungPoint.getHeading(), samples[sampleCounter].getHeading())
                            .build();
                    robot.drive.drive.followPath(pickSample);
                })
                .transition(() -> !robot.drive.drive.isBusy(), blueNetZoneStates.INTAKE)
                .transition(() -> sampleCounter >= 2, blueNetZoneStates.COLLAPSE_TIME)
                .onExit(() -> previousState = blueNetZoneStates.PICK_SAMPLE)

                .state(blueNetZoneStates.DROP_SAMPLE)
                .onEnter(() -> {
                    dropSample =
                            new PathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    samples[sampleCounter].getAsPoint(),
                                                    highBasket.getAsPoint()
                                            )
                                    )
                                    .setLinearHeadingInterpolation(robot.drive.drive.getTotalHeading(), highBasket.getHeading())
                                    .build();
                    robot.outtake.setArm(Outtake.ARM_BASKET);
                    robot.drive.drive.followPath(dropSample);
                    robot.dr4b.setPosition(DR4B.UPPER_BASKET);
                })
                .transition(() -> !robot.drive.drive.isBusy() && sampleCounter < 2, blueNetZoneStates.PICK_SAMPLE)
                .transition(() -> !robot.drive.drive.isBusy() && sampleCounter >= 2, blueNetZoneStates.PARK)
                .onExit(() -> {
                    sampleCounter++;
                    previousState = blueNetZoneStates.DROP_SAMPLE;
                })

                .state(blueNetZoneStates.PARK)
                .onEnter(() -> {
                    robot.drive.drive.followPath(parkAscentZone);
                    robot.outtake.setArm(Outtake.ARM_BASKET);
                    robot.dr4b.setPosition(0);
                })
                .transition(() -> !robot.drive.drive.isBusy(), blueNetZoneStates.STOP)
                .onExit(() -> {
                    previousState = blueNetZoneStates.PARK;
                })

                .state(blueNetZoneStates.STOP)
                .loop(() -> multipleTelemetry.addLine("AUTO COMPLETED"))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        blueNetZoneAuto.start();

        while (opModeIsActive()) {
            multipleTelemetry.addData("Auto State", blueNetZoneAuto.getStateEnum());
            multipleTelemetry.addData("X", robot.drive.drive.getPose().getX());
            multipleTelemetry.addData("Y", robot.drive.drive.getPose().getY());
            multipleTelemetry.addData("Heading", robot.drive.drive.getPose().getHeading());
//            robot.drive.drive.telemetryDebug(multipleTelemetry);
            robot.update();
            blueNetZoneAuto.update();
            multipleTelemetry.update();
            if (blueNetZoneAuto.getStateEnum() == blueNetZoneStates.STOP) {
                blueNetZoneAuto.stop();
                blueNetZoneAuto.reset();
                return;
            }
        }
    }
}
