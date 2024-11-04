package org.firstinspires.ftc.teamcode.CompetitionOpmodes.Autonomous.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;


@Autonomous(name = "Blue Observation Zone")
public class BlueObservationZone extends LinearOpMode {
    Robot robot;
    Pose startPose = new Pose(7.354, 56.224, Math.toRadians(180));
    @Override
    public void runOpMode() {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot = new Robot(hardwareMap, multipleTelemetry, startPose, true);
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(7.354, 56.224, Point.CARTESIAN),
                                new Point(32.264, 63.578, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(32.264, 63.578, Point.CARTESIAN),
                                new Point(10.201, 64.527, Point.CARTESIAN),
                                new Point(27.519, 33.213, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315));

        PathChain chain = builder.build();

        waitForStart();
        if (isStopRequested()) return;

        robot.drive.drive.followPath(chain);
        while (robot.drive.drive.isBusy()) {
            robot.drive.update();
            telemetry.addData("X", robot.drive.drive.getPose().getX());
            telemetry.addData("Y", robot.drive.drive.getPose().getY());
            telemetry.addData("Theta", robot.drive.drive.getPose().getHeading());
            telemetry.update();
        }
    }
}
