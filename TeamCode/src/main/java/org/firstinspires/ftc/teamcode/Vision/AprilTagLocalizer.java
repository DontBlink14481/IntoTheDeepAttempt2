package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

@Config
public class AprilTagLocalizer {
    public static double APRIL_TAG_HEIGHT = 2.953;

    public static Pose localSujay(AprilTagDetection detection) {
        AprilTagLibrary tagData = AprilTagGameDatabase.getCenterStageTagLibrary();
        AprilTagPoseFtc relativePos = detection.ftcPose;
        VectorF fieldPos = tagData.lookupTag(detection.id).fieldPosition;


        double rotAmount = -relativePos.yaw;

        Vector outputVec = new Vector(Math.hypot(-relativePos.x, -relativePos.y), 180 + relativePos.bearing + rotAmount);/*.plus(new Vector(fieldPos.get(0), fieldPos.get(2)));*/

        return new Pose(outputVec, Math.toRadians(
                tagData.lookupTag(detection.id).fieldOrientation
                        .toOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).thirdAngle
                        + rotAmount));


        /*Pose beforeRot = new Pose(fieldPos.get(0) - relativePos.x,
                fieldPos.get(1) - relativePos.y,
                tagData.lookupTag(detection.id).fieldOrientation.toOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES)
                        .thirdAngle);*/
        /*Pose afterRot = new Pose(beforeRot.getX() + Math.hypot(relativePos.x, relativePos.y)*(1-Math.cos(beforeRot.getHeading() + rotAmount)),
                beforeRot.getY() + Math.hypot(relativePos.x, relativePos.y)*(Math.sin(beforeRot.getHeading())+rotAmount),
                beforeRot.getHeading() + rotAmount);*/

//        Vector vec = Vector.polar(Math.cos(Math.toRadians(30)) * relativePos.y + Drivebase.CENTER_DIST, Math.toRadians(-relativePos.yaw + 90)).plus(Vector.polar(relativePos.x, Math.toRadians(-relativePos.yaw)));
        /*Pose newRobotPose = new Pose(
                -tagData.lookupTag(detection.id).fieldPosition.getData()[1] - vec.getX(),
                tagData.lookupTag(detection.id).fieldPosition.getData()[0] - vec.getY(),
                Math.toRadians(-relativePos.yaw + 90)
        );
*/
        /*robot.drivebase.setPoseEstimate(new Pose(
                tagData[id].get() - (Math.signum(aprilTagPose.bearing) * aprilTagPose.x),
                tagData[id].getY() - (Math.signum(aprilTagPose.bearing) * aprilTagPose.y),
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        ));*/
    }



    public static Vector relative(AprilTagDetection det){
        return new Vector(det.ftcPose.x, det.ftcPose.y, false);
    }

    public static double getInitialHeading(AprilTagDetection det){
        return getMetadata(det).fieldOrientation
                .toOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).thirdAngle - 90;
    }

    public static double getFirstAngle(AprilTagDetection det){
        return getMetadata(det).fieldOrientation
                .toOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle;
    }

    public static double getSecondAngle(AprilTagDetection det){
        return getMetadata(det).fieldOrientation
                .toOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).secondAngle;
    }

    public static double getThirdAngle(AprilTagDetection det){
        return getMetadata(det).fieldOrientation
                .toOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).thirdAngle;
    }

    public static AprilTagMetadata getMetadata(AprilTagDetection det){
        return AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(det.id);
    }

    public static Vector yawRotation(AprilTagDetection det){
        double initialHeading = getInitialHeading(det);
        //might have to change to minus yaw
        return new Vector(det.ftcPose.range, Math.toRadians(initialHeading + det.ftcPose.bearing + det.ftcPose.yaw));
    }

    public static Vector getShifted(Vector initial, AprilTagDetection det){
        AprilTagMetadata met = getMetadata(det);
        return initial.plus(new Vector(met.fieldPosition.get(0) - APRIL_TAG_HEIGHT * Math.cos(Math.toRadians(getFirstAngle(det)))
                , met.fieldPosition.get(1), false));
    }

    public static Vector getCenter(Vector shifted, double heading){
        return shifted.minus(new Vector(-Drivebase.CENTER_DIST, 0, false).rotated(heading));
    }

    public static Pose finalShifted(AprilTagDetection det){
        Vector initial = yawRotation(det);
        AprilTagMetadata met = getMetadata(det);
        Vector shifted = getShifted(initial, det);
        Vector center = getCenter(shifted, Math.toRadians(getInitialHeading(det) + det.ftcPose.yaw));
        //might have to change to minus yaw
        return new Pose(center, Math.toRadians(getInitialHeading(det) + det.ftcPose.yaw));
    }

}