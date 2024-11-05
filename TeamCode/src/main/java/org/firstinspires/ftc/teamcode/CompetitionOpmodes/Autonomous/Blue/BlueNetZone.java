package org.firstinspires.ftc.teamcode.CompetitionOpmodes.Autonomous.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Util;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class BlueNetZone extends LinearOpMode {
    Robot robot;

    Pose startPose = new Pose(4.745, 88.250, Math.toRadians(180));
    Point rungPoint = new Point(29.891, 78.761, Point.CARTESIAN);
    Point firstSample = new Point(27.519, 130.241, Point.CARTESIAN);
    Point secondSample;
    Point thirdSample;
    Point highBasket = new Point(17.081, 130.715, Point.CARTESIAN);
    Point ascentZonePark;

    PathChain hangSpeciman;
    PathChain pickFirstSample;
    PathChain dropFirstSample;
    PathChain pickSecondSample;
    PathChain dropSecondSample;
    PathChain pickThirdSample;
    PathChain dropThirdSample;
    PathChain parkAscentZone;

    @Override
    public void runOpMode() {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot = new Robot(hardwareMap, multipleTelemetry, startPose, true);


        waitForStart();
        if (isStopRequested()) return;
        waitForStart();

        while (robot.drive.drive.isBusy()) {
            Util.writePosToFile(robot);
            robot.drive.update();
            telemetry.addData("X", robot.drive.drive.getPose().getX());
            telemetry.addData("Y", robot.drive.drive.getPose().getY());
            telemetry.addData("Theta", robot.drive.drive.getPose().getHeading());
            telemetry.update();
        }
    }

    public void initialize() {
//        hangSpeciman =
//                new PathBuilder()
//                        .addPath()
//                    .build();
    }
}
