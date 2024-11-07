package org.firstinspires.ftc.teamcode.TeleopTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DR4B;


@TeleOp
@Config
public class DR4BTuner extends LinearOpMode {

    public static double position = 0;

    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DR4B d = new DR4B(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("ref", position);
            d.setPosition(position);


            telemetry.addData("real angle", d.getAngle());
            telemetry.addData("real positoin", d.getRealPosition());
            telemetry.addData("poewr", d.getPower());
            telemetry.update();
            d.update();
        }
    }
}
