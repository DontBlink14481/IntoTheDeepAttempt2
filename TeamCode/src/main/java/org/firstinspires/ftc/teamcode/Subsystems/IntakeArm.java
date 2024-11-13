package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Config
public class IntakeArm implements Subsystem {

    public static Servo claw;
    public static Servo armL, armR;
    public static ColorSensor colorSensor;
    public static Servo swivel;

    public static double CLAW_OPEN = 0.2;
    public static double CLAW_GRAB = 0.365;
    public static  double CLAW_SEMI = (CLAW_GRAB + CLAW_OPEN) /2;
    public static double ARM_GRAB = 0.48;
    public static double ARM_TRANSFER = 0.18;
    public static double FLOAT_ARM = 0.42;

    public static double ARM_BUMP = 0.46;

    public static double SWIVEL_FLAT = 0.44;
    public static double SWIVEL_LEFT = 0.8;
    public static double SWIVEL_RIGHT = 0.26;
    public static double ZERO_ANGLE = 0.11;



    public IntakeArm(HardwareMap map) {
//        colorSensor = map.get(ColorSensor.class, "color");
        claw = map.get(Servo.class, "intakeClaw");
        armL = map.get(Servo.class, "leftIntakeArm");
        armR = map.get(Servo.class, "rightIntakeArm");
        armR.setDirection(Servo.Direction.REVERSE);
        swivel = map.get(Servo.class, "intakeSwivel");
    }


    public void setArm(double p) {
        armL.setPosition(p);
        armR.setPosition(p);
    }

    public void setSwivel(double p) {
        swivel.setPosition(p);
    }

    public void setClaw(double p) {
        claw.setPosition(p);
    }

    public Pose pollColor(){
        return new Pose(colorSensor.red(), colorSensor.green(), colorSensor.blue());
    }

    public void grab(){
        claw.setPosition(CLAW_GRAB);
    }

    public void release(){
        claw.setPosition(CLAW_OPEN);
    }

    public static double joystickToSwivel(double x, double y){
        double angle = Math.toDegrees(Math.atan2(-x, y));
        while (angle > 90 || angle < -90){
            angle = angle - Math.signum(angle)*180;
        }
        return swivelAngleToPos(angle);
    }

    public static double swivelAngleToPos(double angle){
        return SWIVEL_FLAT + (SWIVEL_FLAT - ZERO_ANGLE)*angle/90;
    }


    @Override
    public void toInit(){
        setArm(ARM_TRANSFER);
        setSwivel(SWIVEL_FLAT);
    }

    @Override
    public void update() {
    }
}
