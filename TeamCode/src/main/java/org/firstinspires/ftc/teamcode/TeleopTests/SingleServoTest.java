package org.firstinspires.ftc.teamcode.TeleopTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class SingleServoTest extends LinearOpMode {

    public static double servoPosition1 = 0.9;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo s = hardwareMap.get(Servo.class, "servo");
        Servo s2 = hardwareMap.get(Servo.class, "servo2");
        s2.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            s.setPosition(servoPosition1);
            s2.setPosition(servoPosition1);
        }
    }
}
