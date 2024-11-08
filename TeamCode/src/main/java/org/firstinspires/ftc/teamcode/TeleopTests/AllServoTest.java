package org.firstinspires.ftc.teamcode.TeleopTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Config
@TeleOp
public class AllServoTest extends LinearOpMode {

    public static double WRIST_POSE = Outtake.WRIST_TRANSFER;
    public static double OUTTAKE_ARM_POSE = Outtake.ARM_TRANSFER;
    public static double INTAKE_ARM_POSE = IntakeArm.ARM_TRANSFER;
    public static double INTAKE_SWIVEL = IntakeArm.SWIVEL_FLAT;
    public static double INTAKE_CLAW = IntakeArm.CLAW_OPEN;
    public static double OUTTAKE_CLAW = Outtake.CLAW_RELEASE;
    public static double OUTTAKE_SWIVEL_POSE = Outtake.SWIVEL_FLAT;



    @Override
    public void runOpMode() throws InterruptedException {

        Outtake outtake = new Outtake(hardwareMap);
        IntakeArm intake = new IntakeArm(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            outtake.setArm(OUTTAKE_ARM_POSE);
            outtake.setWrist(WRIST_POSE);
            outtake.setSwivel(OUTTAKE_SWIVEL_POSE);
            outtake.setClaw(OUTTAKE_CLAW);

            intake.setArm(INTAKE_ARM_POSE);
            intake.setSwivel(INTAKE_SWIVEL);
            intake.setClaw(INTAKE_CLAW);
        }
    }
}
