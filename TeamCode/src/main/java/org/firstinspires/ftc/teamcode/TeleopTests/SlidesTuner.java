package org.firstinspires.ftc.teamcode.TeleopTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DR4B;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSlides;


@TeleOp
@Config
public class SlidesTuner extends LinearOpMode {

    public static double position = 0;

    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IntakeSlides d = new IntakeSlides(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("ref", position);
            d.setPosition(position);

            telemetry.addData("real angle", d.getAngle());
            telemetry.addData("real pos", d.getRealPosition());
            telemetry.update();
            d.update();
        }
    }
}
