package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagLocalizer extends Localizer {
    @Config
    static class Params {
        // distance FROM robot center TO camera (inches)
        // TODO: tune
        static Vector cameraOffset = new Vector(
                0,
                0);

    }
    private HardwareMap hardwareMap;
    private Vector cameraOffset;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    Pose localizerPose = new Pose();
    long tagDetectTime;
    List<AprilTagDetection> detections;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 0, -90, 0);

    public AprilTagLocalizer(HardwareMap map, AprilTagProcessor aprilTag) {
        this(map, new Pose(), aprilTag);
    }

    public AprilTagLocalizer(HardwareMap map, Pose setStartPose, AprilTagProcessor aprilTag) {
        hardwareMap = map;
        aprilTagProcessor = aprilTag;
        this.cameraOffset = Params.cameraOffset;

    }

    @Override
    public void update() {
        detections = aprilTagProcessor.getFreshDetections();
        if (detections != null && !detections.isEmpty() && detections.get(0) != null) {
            tagDetectTime = detections.get(0).frameAcquisitionNanoTime;
            AprilTagDetection detection = detections.get(0);
            double x = detection.robotPose.getPosition().x;
            double y = detection.robotPose.getPosition().y;
            double heading = Math.toRadians(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES) + 90);

            localizerPose = new Pose(x, y, heading);
        }
    }

    @Override
    public Pose getPose() {
        return localizerPose;
    }

    @Override
    public Pose getVelocity() {
        return null;
    }

    @Override
    public Vector getVelocityVector() {
        return null;
    }

    @Override
    public void setStartPose(Pose setStart) {
        localizerPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        localizerPose = setPose;
    }



    @Override
    public double getTotalHeading() {
        return localizerPose.getHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return 0;
    }

    @Override
    public double getLateralMultiplier() {
        return 0;
    }

    @Override
    public double getTurningMultiplier() {
        return 0;
    }

    @Override
    public void resetIMU() {

    }
}
