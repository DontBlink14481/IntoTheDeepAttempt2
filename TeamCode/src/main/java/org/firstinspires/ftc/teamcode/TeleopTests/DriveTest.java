package org.firstinspires.ftc.teamcode.TeleopTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.TeleControl.DriveControl;

@TeleOp
@Config
public class DriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Drivebase d = new Drivebase(hardwareMap);

        DriveControl dc = new DriveControl(d, gamepad1, gamepad2);

        waitForStart();
        while(opModeIsActive()){
            dc.update();
            telemetry.addData("left stick x", gamepad1.left_stick_x);
            telemetry.addData("left stick y]", gamepad1.left_stick_y);
            telemetry.addData("right stick x", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}
