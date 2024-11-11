package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Util.MotionProfile;
import org.firstinspires.ftc.teamcode.Util.Util;

@Config
public class Outtake implements Subsystem {

    public static double SWIVEL_FLAT = 0.5 ;
    public static double SWIVEL_OUTTAKE = SWIVEL_FLAT;
    public static double SWIVEL_180 = 0.75;
    HardwareMap hardwareMap;
    public Servo armLeft;
    public Servo armRight;
    public Servo swivel;
    public Servo claw;

    public static double ARM_TRANSFER = 0.23;
    public static double ARM_FLAT_IN = 0.15;
    public static double ARM_FLAT_OUT = 0.68;
    public static double ARM_MAX_POS = armAngleToPos(90+25);


    public int swivelPos = 0; // by default at flat position

    // wrist stuff
    public Servo wrist; // up & down tilt
    public static double WRIST_TRANSFER =  .15; //.15
    public static double WRIST_OVER_90 =  .53; //.15 CHANGENNGEGE
    public static double WRIST_MID = 0.25;
    public static double WRIST_OVER_90F = 0;
    public static double WRIST_MAX = 1;
    public static double BASKET_ANGLE = 45;

    public static double CLAW_RELEASE = 0.6;
    public static double CLAW_GRAB = 0.1;

    private static final double armPosToAngle = 180/(ARM_FLAT_OUT - ARM_FLAT_IN); // TODO: update); // TODO; update
    private static final double wristAngleToPos = (WRIST_OVER_90 - WRIST_MID)/90; // TODO: update

    public static double ARM_BASKET = 0.4;
    public static double ARM_SPECIMEN = 0.5;
    public static double ARM_OBSERVATION = ARM_BASKET;

    public static double WRIST_BASKET = 0.42;
    public static double WRIST_SPECIMEN = 0.6;
    public static double WRIST_OBSERVATION = WRIST_BASKET;


    public static double ARM_OUT_INTER = 0.3;
    public static double WRIST_OUT_INTER = 0.1;


    //arm mp stuff
    public double armSetPosition = -1;
    public double armStartTime = -1;
    public double armStartPos = -1;
    public static double armMPCut = 0.02;
    public static double m = 0.5;
    public static double c = 0.07;

    public Outtake(HardwareMap h) {
        hardwareMap = h;
        armLeft = hardwareMap.get(Servo.class, "leftOuttake");
        armRight = hardwareMap.get(Servo.class, "rightOuttake");
        armRight.setDirection(Servo.Direction.REVERSE);
        swivel = hardwareMap.get(Servo.class, "outtakeSwivel");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "outtakeClaw");
    }


    public void setArm(double pos) {
        if(pos > ARM_MAX_POS) pos = ARM_MAX_POS;
        armLeft.setPosition(pos);
        armRight.setPosition(pos);
    }

    public void setClaw(double pose) {
        claw.setPosition(pose);
    }

    public void setWrist(double pos) {
        wrist.setPosition(pos);
    }

    public void setSwivel(double pos) {
        swivel.setPosition(pos);
    }

    public void grab() {
        claw.setPosition(CLAW_GRAB);
    }

    public void release() {
        claw.setPosition(CLAW_RELEASE);
    }

    public void outtake() {
        setSwivel(SWIVEL_OUTTAKE);
        setArm(ARM_BASKET);
        setWrist(WRIST_BASKET);
    }

    public void toInit() {
        setSwivel(SWIVEL_OUTTAKE);
        outtakeInter();
        release();
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

    public void transfer(){
        setArm(ARM_TRANSFER);
        setSwivel(SWIVEL_OUTTAKE);
        setWrist(WRIST_TRANSFER);
        release();
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

    public static double armAngleToPos(double angle) {
        return angle / armPosToAngle + ARM_FLAT_IN;
    }

    public double  getGoodWristPosition(double goalAngle) {
        return wristAngleToPos(armPosToAngle(armLeft.getPosition()) + goalAngle);
    }

    public double wristAngleToPos(double baseAngle) {
        double wristPos = WRIST_MID + baseAngle * wristAngleToPos;
        return Math.max(wristPos, WRIST_MAX);
    }

    public void outtakeInter(){
        setArm(ARM_OUT_INTER);
        setWrist(WRIST_OUT_INTER);
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
