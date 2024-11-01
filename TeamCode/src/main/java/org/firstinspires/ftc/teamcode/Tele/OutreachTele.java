package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.TeleControl.DriveControl;

@TeleOp
public class OutreachTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drivebase d = new Drivebase(hardwareMap);
        DriveControl dc = new DriveControl(d, gamepad1, gamepad2);
        dc.outreachMode = true;
        waitForStart();

        while(opModeIsActive()){
            dc.update();
        }

    }
}
