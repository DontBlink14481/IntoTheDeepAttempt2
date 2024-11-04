package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;


@Autonomous
public class AutoTest extends LinearOpMode {

    Follower follower;
    Path testPath;

    @Override
    public void runOpMode() {
        follower = new Follower(hardwareMap);
        testPath = new Path(new BezierCurve(new Point(0, 0, Point.CARTESIAN),
                new Point(0, 0, Point.CARTESIAN),
                new Point(50, 50, Point.CARTESIAN),
                new Point(60, 20, Point.CARTESIAN)));

        telemetry.addData("Status", "INIT");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        telemetry.addData("Status", "Following Path");
        telemetry.update();

        follower.followPath(testPath);
        while (follower.isBusy()) {
            follower.update();
        }
    }
}
