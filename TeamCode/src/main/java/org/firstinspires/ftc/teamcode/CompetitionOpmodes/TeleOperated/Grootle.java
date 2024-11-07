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
        NEUTRAL, INTAKE, TRANSFER, COLLAPSE, OUTTAKE
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot r = new Robot(hardwareMap, telemetry);

        StateMachine transferMachine = StateMachines.getTransferMachine(r, telemetry);
        StateMachine collapseMachine = StateMachines.getCollapseMachine(r, telemetry);

        DriveControl dc = new DriveControl(r, gamepad1, gamepad2);
        IntakeControl ic = new IntakeControl(r, gamepad1, gamepad2);
        OuttakeControl oc = new OuttakeControl(r, gamepad1, gamepad2);
        DR4BControl drbc = new DR4BControl(r, gamepad1, gamepad2);

        StateMachine machine = new StateMachineBuilder()
                .state(TeleStates.NEUTRAL)
                .onEnter(r::toInit)
                .transition(() -> gamepad2.right_trigger > 0.1, TeleStates.INTAKE)

                .state(TeleStates.INTAKE)
                .onEnter(() -> {
                    r.intakeSlides.setPosition(IntakeSlides.PARTIAL);
                    r.intakeArm.release();
                })
                .loop(ic::update)
                .onExit(() -> {
                    r.intakeSlides.setPosition(IntakeSlides.IN);
                    r.intakeArm.setArm(IntakeArm.FLOAT_ARM);
                    ic.armUp = true;
                })
                .transition(() -> gamepad2.cross, TeleStates.TRANSFER)

                .state(TeleStates.TRANSFER)
                .onEnter(transferMachine::start)
                .loop(transferMachine::update)
                .transition(() -> (transferMachine.getState() == StateMachines.TransferStates.FINISHED) && gamepad2.dpad_up)
                .onExit(() -> {
                    transferMachine.stop();
                    transferMachine.reset();
                })

                .state(TeleStates.OUTTAKE)
                .loop(() -> {
                    r.outtake.outtake();
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

        waitForStart();

        machine.start();
        while (opModeIsActive()) {
            machine.update();
            dc.update();
            r.update();
        }
    }
}
