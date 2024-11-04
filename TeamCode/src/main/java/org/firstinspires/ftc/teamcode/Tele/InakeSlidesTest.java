package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSlides;


@TeleOp
@Config
public class InakeSlidesTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        IntakeSlides intakeSlides = new IntakeSlides(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            intakeSlides.setRawPower(gamepad1.left_stick_y);
        }
    }
}
