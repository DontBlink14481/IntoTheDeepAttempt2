package org.firstinspires.ftc.teamcode.CompetitionOpmodes.Autonomous.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
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

@Autonomous
public class BlueNetZone extends LinearOpMode {
    Robot robot;
    MultipleTelemetry telemetry;
    StateMachine blueNetZoneAuto;
    StateMachine collapseMachine;
    StateMachine transferMachine;
    StateMachine intakeMachine;

    public static Pose startPose = (new Pose(4.745, 88.250, Math.toRadians(180))).convertToFTC();
    public static Pose rungPoint = (new Pose(29.891, 78.761, Math.toRadians(180))).convertToFTC();
    public static Pose firstSample = (new Pose(28.468, 121.226, Math.toRadians(20))).convertToFTC();
    public static Pose secondSample = (new Pose(27.519, 130.715, Math.toRadians(0))).convertToFTC();
    public static Pose thirdSample = (new Pose(25.621, 135.222, Math.toRadians(15))).convertToFTC();
    public static Pose highBasket = (new Pose(17.081, 130.715, Math.toRadians(315))).convertToFTC();
    public static Pose ascentZonePark = (new Pose(63.815, 94.418, Math.toRadians(90))).convertToFTC();
    public static Point constBezierCurvePoint = new Point(4.270, 114.820, Point.CARTESIAN);


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
        initialize();

        waitForStart();
        if (isStopRequested()) return;

        blueNetZoneAuto.start();

        while (robot.drive.drive.isBusy()) {
            Util.writePosToFile(robot);
            telemetry.addData("Auto State", blueNetZoneAuto.getStateEnum());

            robot.update();
            blueNetZoneAuto.update();
            telemetry.update();
            if (blueNetZoneAuto.getStateEnum() == blueNetZoneStates.STOP) {
                blueNetZoneAuto.stop();
                return;
            }
        }
    }

    public void initialize() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot = new Robot(hardwareMap, telemetry, startPose, true, gamepad1, gamepad2);
        collapseMachine = StateMachines.getCollapseMachine(robot, telemetry);
        transferMachine = StateMachines.getTransferMachine(robot, telemetry);
        intakeMachine = StateMachines.getIntakingMachine(robot, telemetry);

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
                })
                .transition(() -> !robot.drive.drive.isBusy(), blueNetZoneStates.COLLAPSE)
                .onExit(() -> {
                    robot.drive.drive.holdCurrentPosition();
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
                    Point currentPoint = new Point(robot.drive.drive.getPose().getX(), robot.drive.drive.getPose().getY());
                    if (currentPoint.distanceFrom(firstSample.getAsPoint()) <= 20) {
                        robot.intakeSlides.setPosition(IntakeSlides.PARTIAL);
                    }
                })
                .transition(() -> !robot.drive.drive.isBusy(), blueNetZoneStates.INTAKE)
                .onExit(() -> previousState = blueNetZoneStates.PICK_SAMPLE)

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
                                    .setLinearHeadingInterpolation(samples[sampleCounter].getHeading(), highBasket.getHeading())
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
                    robot.drive.drive.holdCurrentPosition();
                    previousState = blueNetZoneStates.PARK;
                })

                .state(blueNetZoneStates.STOP)
                .loop(() -> telemetry.addLine("AUTO COMPLETED"))
                .build();

    }
}
