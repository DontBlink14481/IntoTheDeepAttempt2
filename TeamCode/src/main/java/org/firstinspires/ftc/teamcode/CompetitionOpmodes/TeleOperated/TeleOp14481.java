package org.firstinspires.ftc.teamcode.CompetitionOpmodes.TeleOperated;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.TeleControl.DR4BControl;
import org.firstinspires.ftc.teamcode.TeleControl.DriveControl;
import org.firstinspires.ftc.teamcode.TeleControl.IntakeControl;
import org.firstinspires.ftc.teamcode.TeleControl.OuttakeControl;
import org.firstinspires.ftc.teamcode.Util.Util;


@TeleOp
public class TeleOp14481 extends LinearOpMode {


    boolean initFlag;
    public Robot robot = null;
    public double lastTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot = new Robot(hardwareMap, multipleTelemetry, Util.getPoseFromFile(), false);

        initFlag = true;
        ElapsedTime t = new ElapsedTime();

        DriveControl dc = new DriveControl(robot, gamepad1, gamepad2);
        IntakeControl ic = new IntakeControl(robot, gamepad1, gamepad2);
        OuttakeControl oc = new OuttakeControl(robot, gamepad1, gamepad2);
        DR4BControl drbc = new DR4BControl(robot, gamepad1, gamepad2);


        waitForStart();

        while (opModeIsActive()) {

            dc.update();
        }
    }
}
