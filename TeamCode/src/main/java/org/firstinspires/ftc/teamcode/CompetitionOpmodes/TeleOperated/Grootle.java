package org.firstinspires.ftc.teamcode.CompetitionOpmodes.TeleOperated;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.Subsystems.DR4B;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.TeleControl.Control;
import org.firstinspires.ftc.teamcode.TeleControl.DR4BControl;
import org.firstinspires.ftc.teamcode.TeleControl.DriveControl;
import org.firstinspires.ftc.teamcode.TeleControl.IntakeControl;
import org.firstinspires.ftc.teamcode.TeleControl.OuttakeControl;
import org.firstinspires.ftc.teamcode.Util.StateMachines;

// our controller operated mode

@TeleOp
@Config
public class Grootle extends LinearOpMode {

    public enum TeleStates {
        NEUTRAL, INTAKE, TRANSFER, COLLAPSE, INTAKE_2, INTAKING_MACHINE, OUTTAKE
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot r = new Robot(hardwareMap, telemetry);

        StateMachine transferMachine = StateMachines.getTransferMachine(r, telemetry);
        StateMachine collapseMachine = StateMachines.getCollapseMachine(r, telemetry);
        StateMachine intakingMachine = StateMachines.getIntakingMachine(r, telemetry);

        DriveControl dc = new DriveControl(r, gamepad1, gamepad2);
        IntakeControl ic = new IntakeControl(r, gamepad1, gamepad2);
        OuttakeControl oc = new OuttakeControl(r, gamepad1, gamepad2);
        DR4BControl drbc = new DR4BControl(r, gamepad1, gamepad2);

        StateMachine machine = new StateMachineBuilder()
                .state(TeleStates.NEUTRAL)
                .onEnter(() -> {
                    r.toInit();
                    r.intakeArm.setArm(IntakeArm.FLOAT_ARM);
                })
                .transition(() -> gamepad2.right_trigger > 0.1, TeleStates.INTAKE_2)

                // float
                .state(TeleStates.INTAKE_2)
                .onEnter(() -> {
                    r.intakeArm.setArm(IntakeArm.FLOAT_ARM);
//                    r.intakeSlides.setPosition(IntakeSlides.PARTIAL);
                    r.intakeArm.release();
                })
                .loop(ic::update)
                .transition(()-> gamepad2.cross)

                .state(TeleStates.INTAKING_MACHINE)
                .onEnter(intakingMachine::start)
                .loop(intakingMachine::update)
                .onExit(() -> {
                    intakingMachine.reset();
                    intakingMachine.stop();
                })
                .transition(() -> intakingMachine.getState() == StateMachines.IntakingStates.FINAL && gamepad2.cross, TeleStates.TRANSFER)
                .transition(() -> intakingMachine.getState() == StateMachines.IntakingStates.FINAL && gamepad2.triangle, TeleStates.INTAKE_2)


               /* .state(TeleStates.INTAKE)
                .onEnter(() -> {
                    r.intakeSlides.setPosition(IntakeSlides.PARTIAL);
                    r.intakeArm.release();

                })
                .loop(ic::update)
                .onExit(() -> {
                    ic.armUp = true;
                })
                .transition(() -> gamepad2.cross, TeleStates.TRANSFER)*/

                .state(TeleStates.TRANSFER)
                .onEnter(transferMachine::start)
                .loop(transferMachine::update)
                .transition(() -> (transferMachine.getState() == StateMachines.TransferStates.FINISHED) && (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right))
                .transition(() -> gamepad2.triangle, TeleStates.INTAKE_2, () ->{
//                    r.intakeSlides.setPosition(IntakeSlides.PARTIAL);
                    r.outtake.toInit();
                })
                .onExit(() -> {
                    transferMachine.stop();
                    transferMachine.reset();
                })

                .state(TeleStates.OUTTAKE)
                .loop(() -> {
                    oc.update();
                    drbc.update();
                })
                .transition(() -> gamepad2.square && !drbc.outtakeOnRung, TeleStates.NEUTRAL)
                .transition(() -> gamepad2.square && drbc.outtakeOnRung, TeleStates.COLLAPSE)

                .state(TeleStates.COLLAPSE)
                .onEnter(collapseMachine::start)
                .loop(collapseMachine::update)
                .onExit(() -> {
                    collapseMachine.stop();
                    collapseMachine.reset();
                    drbc.outtakeOnRung = false;
                })
                .transition(() -> (collapseMachine.getState() == StateMachines.CollapseStates.FINISHED), TeleStates.NEUTRAL)
                .build();

        r.toInit();
        waitForStart();

        machine.start();
        while (opModeIsActive()) {
            telemetry.addData("Tele State", machine.getState());
            telemetry.addData("Intake slide SET spositoin", r.intakeSlides.position);
            telemetry.addData("Intake slide curr position", r.intakeSlides.getRealPosition());
            telemetry.update();

            machine.update();
            dc.update();
            r.update();
        }
    }
}
