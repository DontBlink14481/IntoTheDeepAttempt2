package org.firstinspires.ftc.teamcode.CompetitionOpmodes.Autonomous.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous(name = "Blue Net Zone")
public class BlueNetZone extends LinearOpMode {
    Robot robot;
    Pose startPose = new Pose(4.745, 88.250, Math.toRadians(180));
    @Override
    public void runOpMode() {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot = new Robot(hardwareMap, multipleTelemetry, startPose, true);
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(4.745, 88.250, Point.CARTESIAN),
                                new Point(29.891, 78.761, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(29.891, 78.761, Point.CARTESIAN),
                                new Point(4.270, 114.820, Point.CARTESIAN),
                                new Point(27.519, 130.241, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(27.519, 130.241, Point.CARTESIAN),
                                new Point(17.081, 130.715, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(17.081, 130.715, Point.CARTESIAN),
                                new Point(69.272, 102.722, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation();

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
