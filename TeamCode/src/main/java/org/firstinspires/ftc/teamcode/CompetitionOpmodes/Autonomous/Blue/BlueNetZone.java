package org.firstinspires.ftc.teamcode.CompetitionOpmodes.Autonomous.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CompetitionOpmodes.TeleOperated.Grootle;
import org.firstinspires.ftc.teamcode.Subsystems.DR4B;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.TeleControl.IntakeControl;
import org.firstinspires.ftc.teamcode.Util.StateMachines;
import org.firstinspires.ftc.teamcode.Util.Util;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class BlueNetZone extends LinearOpMode {
    Robot robot;
    StateMachine blueNetZoneAuto;
    StateMachine collapseMachine;
    StateMachine transferMachine;
    StateMachine intakeMachine;

    MultipleTelemetry multipleTelemetry;

    public static Pose startPose = (new Pose(9.015, 88.488, Math.toRadians(180))).convertToFTC();
    public static Pose rungPoint = (new Pose(25.110, 82.082, Math.toRadians(180))).convertToFTC();
    public static Pose firstSample = (new Pose(28.468, 121.226, Math.toRadians(40))).convertToFTC();
    public static Pose secondSample = (new Pose(27.519, 130.715, Math.toRadians(0))).convertToFTC();
    public static Pose thirdSample = (new Pose(25.621, 135.222, Math.toRadians(15))).convertToFTC();
    public static Pose highBasket = (new Pose(17.081, 130.715, Math.toRadians(315))).convertToFTC();
    public static Pose ascentZonePark = (new Pose(63.815, 94.418, Math.toRadians(90))).convertToFTC();
    public static Point constBezierCurvePoint = new Point(5, 108.178, Point.CARTESIAN).convertToFTC();


    Pose[] samples = {firstSample, secondSample, thirdSample};

    PathChain hangSpeciman;
    PathChain pickSample;
    PathChain dropSample;
    PathChain travelToSample;
    PathChain parkAscentZone;

    int sampleCounter = 0;
    boolean startAuto = false;
    enum blueNetZoneStates{
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

        hangSpeciman =
                new PathBuilder()
                        .addPath(
                                new BezierLine(startPose.getAsPoint(),
                                        rungPoint.getAsPoint()))
                        .setConstantHeadingInterpolation(rungPoint.getHeading())
                        .build();
        travelToSample =
                new PathBuilder()
                        .addPath(
                                new BezierCurve(rungPoint.getAsPoint(),
                                        constBezierCurvePoint,
                                        firstSample.getAsPoint()))
                        .setLinearHeadingInterpolation(rungPoint.getHeading(), firstSample.getHeading())
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
                .transition(() -> startAuto, blueNetZoneStates.HANG_SPECIMAN)
                .onExit(() -> previousState = blueNetZoneStates.START)

                .state(blueNetZoneStates.HANG_SPECIMAN)
                .onEnter(() -> {
                    robot.dr4b.setPosition(DR4B.UPPER_SPECIMEN);
                    robot.drive.drive.followPath(hangSpeciman);
                    robot.outtake.setArm(Outtake.ARM_SPECIMEN);
                    robot.outtake.setWrist(Outtake.WRIST_SPECIMEN);
                })
                .transition(() -> !robot.drive.drive.isBusy(), blueNetZoneStates.COLLAPSE)
                .onExit(() -> {
                    previousState = blueNetZoneStates.HANG_SPECIMAN;
                })

                .state(blueNetZoneStates.COLLAPSE)
                .onEnter(() -> collapseMachine.start())
                .loop(() -> collapseMachine.update())
                .transition(() -> collapseMachine.getStateEnum() == StateMachines.CollapseStates.FINISHED && sampleCounter >= 2, blueNetZoneStates.PARK)
                .transition(() -> collapseMachine.getStateEnum() == StateMachines.CollapseStates.FINISHED && previousState == blueNetZoneStates.HANG_SPECIMAN, blueNetZoneStates.TRAVEL_TO_SAMPLES)
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
                    if (currentPoint.distanceFrom(firstSample.getAsPoint()) <= 5 && robot.drive.drive.getPose().getHeading() <= Math.toRadians(50)) {
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
                .transition(() -> sampleCounter >= 2, blueNetZoneStates.COLLAPSE)
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
