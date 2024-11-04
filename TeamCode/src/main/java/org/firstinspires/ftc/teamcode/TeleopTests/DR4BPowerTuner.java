package org.firstinspires.ftc.teamcode.TeleopTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DR4B;

@TeleOp
@Config
public class DR4BPowerTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DR4B d = new DR4B(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            d.setUnrestrictedRawPower(gamepad1.left_stick_y);
        }
    }
}
