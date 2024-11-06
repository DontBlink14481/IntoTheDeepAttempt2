package org.firstinspires.ftc.teamcode.TeleopTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class OuttakeServoTest extends LinearOpMode {

    public static double WRIST_POSE = 0;
    public static double ARM_POSE = 0.4;
    public static double OUTTAKE_CLAW = 0;
    public static double SWIVEL_POSE = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo armL = hardwareMap.servo.get("leftOuttake");
        Servo armR = hardwareMap.servo.get("rightOuttake");
        armR.setDirection(Servo.Direction.REVERSE);
        Servo swivel = hardwareMap.servo.get("outtakeSwivel");
        Servo claw = hardwareMap.servo.get("outtakeClaw");
        Servo wrist = hardwareMap.servo.get("wrist");

        waitForStart();
        while (opModeIsActive()) {
            armL.setPosition(ARM_POSE);
            armR.setPosition(ARM_POSE);
            swivel.setPosition(SWIVEL_POSE);
            claw.setPosition(OUTTAKE_CLAW);
            wrist.setPosition(WRIST_POSE);
        }
    }
}
