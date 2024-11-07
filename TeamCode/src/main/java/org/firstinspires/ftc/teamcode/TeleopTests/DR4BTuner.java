package org.firstinspires.ftc.teamcode.TeleopTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DR4B;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


@TeleOp
@Config
public class DR4BTuner extends LinearOpMode {

    public static double position = 0;
    public static double ARM_POSE = Outtake.ARM_SPECIMEN;
    public static double ARM_WRIST = Outtake.WRIST_SPECIMEN;
    public static double CLAW = Outtake.CLAW_RELEASE;

    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DR4B d = new DR4B(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("ref", position);
            d.setPosition(position);
            outtake.setArm(ARM_POSE);
            outtake.setWrist(ARM_WRIST);
            outtake.setClaw(CLAW);

            telemetry.addData("real angle", d.getAngle());
            telemetry.addData("real positoin", d.getRealPosition());
            telemetry.addData("poewr", d.getPower());
            telemetry.update();
            d.update();
        }
    }
}
