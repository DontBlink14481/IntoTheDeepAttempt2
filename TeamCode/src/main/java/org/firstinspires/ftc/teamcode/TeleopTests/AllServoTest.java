package org.firstinspires.ftc.teamcode.TeleopTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Config
@TeleOp
public class AllServoTest extends LinearOpMode {

    public static double WRIST_POSE = 0.2;
    public static double OUTTAKE_ARM_POSE = 0.5;
    public static double INTAKE_ARM_POSE = 0.5;
    public static double INTAKE_SWIVEL = 0.5;
    public static double INTAKE_CLAW = 0.2;
    public static double OUTTAKE_CLAW = 0.3;
    public static double SWIVEL_POSE = 0.3;



    @Override
    public void runOpMode() throws InterruptedException {

        Outtake outtake = new Outtake(hardwareMap);
        IntakeArm intake = new IntakeArm(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            outtake.setArm(OUTTAKE_ARM_POSE);
            outtake.setWrist(WRIST_POSE);
            outtake.setSwivel(SWIVEL_POSE);
            outtake.setClaw(OUTTAKE_CLAW);

            intake.setArm(INTAKE_ARM_POSE);
            intake.setSwivel(INTAKE_SWIVEL);
            intake.setClaw(INTAKE_CLAW);
        }
    }
}
