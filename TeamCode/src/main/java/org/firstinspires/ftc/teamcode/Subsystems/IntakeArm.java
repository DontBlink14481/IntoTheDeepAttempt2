package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Config
public class IntakeArm implements Subsystem {

    public static Servo claw;
    public static Servo armL, armR;
    public static ColorSensor colorSensor;
    public static Servo swivel;

    public static double ARM_OUTTAKE = 0.3;
    public static double CLAW_OPEN = 0.5;



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


    @Override
    public void toInit(){

    }

    @Override
    public void update() {
    }
}
