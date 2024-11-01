package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Util.MotionProfile;
import org.firstinspires.ftc.teamcode.Util.Util;

@Config
public class Outtake implements Subsystem {

    public static double SWIVEL_TRANSFER = 0.49 ;
    public static double SWIVEL_OUTTAKE = 0.552;
    public static double SWIVEL_180 = 0.75;
    HardwareMap hardwareMap;
    public Servo armLeft;
    public Servo armRight;
    public Servo swivel;
    public Servo claw;

    public static double ARM_PRETRANSFER = 0.23; // default arm outtake pos // .21
    public static double ARM_TRANSFER = 0.1;
    public static double ARM_TRANSFER_DOWN = 0;
    public static double ARM_FLAT_IN = 0.225;
    public static double ARM_FLAT_OUT = 0.72;
    public int swivelPos = 0; // by default at flat position

    // wrist stuff
    public Servo wrist; // up & down tilt
    public static double WRIST_TRANSFER = 0.54; //.15
    public static double WRIST_PRETRANSFER = 0.57; // .55
     public double WRIST_MID = 0;
     public double WRIST_OVER_90 = 0;
    public double WRIST_MAX = 1;

    private final double armPosToAngle = 180/(ARM_FLAT_OUT - ARM_FLAT_IN); // TODO: update); // TODO; update
    private final double wristAngleToPos = (WRIST_OVER_90 - WRIST_MID)/90; // TODO: update


    public static double ARM_BASKET = 0.1;
    public static double ARM_SPECIMEN = 0.1;

    public static double WRIST_BASKET = 0.1;
    public static double WRIST_SPECIMEN = 0.1;



    //arm mp stuff
    public double armSetPosition = -1;
    public double armStartTime = -1;
    public double armStartPos = -1;
    public static double armMPCut = 0.02;
    public static double m = 0.5;
    public static double c = 0.07;

    public Outtake(HardwareMap h) {
        hardwareMap = h;
        armLeft = hardwareMap.get(Servo.class, "outL");
        armRight = hardwareMap.get(Servo.class, "outR");
        swivel = hardwareMap.get(Servo.class, "swivel");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
    }


    public void setArm(double pos) {
        armLeft.setPosition(pos);
        armRight.setPosition(1 - pos);
    }

    public void setWrist(double pos) {
        wrist.setPosition(pos);
    }

    public void setSwivel(double pos) {
        swivel.setPosition(pos);
    }


    public void toInit() {
        setSwivel(SWIVEL_TRANSFER);
        setArm(ARM_PRETRANSFER);
        setWrist(WRIST_PRETRANSFER);
    }

    @Override
    public void update() {
        //arm mp
        if(armSetPosition != -1){
            if(Util.isCloseEnough(armLeft.getPosition(), armSetPosition, armMPCut)){
                cancelArmMP(armSetPosition);
            }
            else{
                setArm(MotionProfile.armMP(armStartTime, armStartPos, armSetPosition).goalPosition);
            }
        }
    }

    public static double swivelAngleToPos(double angle){
        return angle*((SWIVEL_OUTTAKE-SWIVEL_180)/180);
    }

    public static double joystickToSwivel(double x, double y){
        double angle = Range.scale(Math.toDegrees(Math.atan2(x, y)) - 90, -270, 90, -180, 180);
        return swivelAngleToPos(angle + ((angle < 0) ? 180 : 0));
    }


    public double armPosToAngle(double pos) {
        return ((pos - ARM_FLAT_IN) * armPosToAngle);
    }

    public double armAngleToPos(double angle) {
        return angle / armPosToAngle + ARM_FLAT_IN;
    }

    public double  getGoodWristPosition(double v4bPos) {
        return wristAngleToPos(armPosToAngle(armLeft.getPosition()));
    }

    public double wristAngleToPos(double baseAngle) {
        double wristPos = WRIST_MID + baseAngle * wristAngleToPos;
        return Math.max(wristPos, WRIST_MAX);
    }



    public void setArmPoseMP(double pos){
        if(pos == armSetPosition) return;
        armStartPos = armLeft.getPosition();
        armSetPosition = pos;
        armStartTime = System.nanoTime()/1E9;
    }

    public void cancelArmMP(double d){
        armSetPosition = -1;
        armStartTime = -1;
        armStartPos = -1;
        setArm(d);
    }
}
