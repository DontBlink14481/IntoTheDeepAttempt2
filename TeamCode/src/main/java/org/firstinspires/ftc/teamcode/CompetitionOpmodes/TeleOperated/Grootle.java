package org.firstinspires.ftc.teamcode.CompetitionOpmodes.TeleOperated;

import static com.fasterxml.jackson.databind.type.LogicalType.Map;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.Subsystems.DR4B;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.TeleControl.Control;
import org.firstinspires.ftc.teamcode.TeleControl.DR4BControl;
import org.firstinspires.ftc.teamcode.TeleControl.DriveControl;
import org.firstinspires.ftc.teamcode.TeleControl.IntakeControl;
import org.firstinspires.ftc.teamcode.TeleControl.OuttakeControl;
import org.firstinspires.ftc.teamcode.Util.StateMachines;

import java.util.HashMap;
import java.util.Map;

// our controller operated mode

@TeleOp
@Config
public class Grootle extends LinearOpMode {

    public enum TeleStates {
        NEUTRAL, INTAKE, TRANSFER, COLLAPSE, INTAKE_2, INTAKING_MACHINE, DROP, OUTTAKE
    }

    public static double DROP_DELAY = 0.3;
    private HashMap<Boolean, String> buttonMap;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot r = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);


        StateMachine transferMachine = StateMachines.getTransferMachine(r, telemetry);
        StateMachine collapseMachine = StateMachines.getCollapseMachine(r, telemetry);
        StateMachine intakingMachine = StateMachines.getIntakingMachine(r, telemetry);

        DriveControl dc = new DriveControl(r, gamepad1, gamepad2);
        IntakeControl ic = new IntakeControl(r, gamepad1, gamepad2);
        OuttakeControl oc = new OuttakeControl(r, gamepad1, gamepad2);
        DR4BControl drbc = new DR4BControl(r, gamepad1, gamepad2);

        // (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.touchpad)
        buttonMap = new HashMap<Boolean, String>();
        buttonMap.put(gamepad1.dpad_up, "gamepad2.dpad_up");
        buttonMap.put(gamepad1.dpad_down, "gamepad2.dpad_down");
        buttonMap.put(gamepad1.dpad_left, "gamepad2.dpad_left");
        buttonMap.put(gamepad2.dpad_right, "gamepad2.dpad_right");
        buttonMap.put(gamepad1.touchpad, "gamepad1.touchpad");


        StateMachine machine = new StateMachineBuilder()
                .state(TeleStates.NEUTRAL)
                .onEnter(() -> {
                    r.toInit();
                })
                .loop(() -> {
                    r.intakeArm.setArm(IntakeArm.FLOAT_ARM);
                })
                .transition(() -> gamepad2.right_bumper || gamepad2.left_bumper || gamepad2.right_stick_y != 0 || gamepad2.left_trigger > 0.1 || gamepad2.right_trigger > 0.1 || gamepad2.cross, TeleStates.INTAKE_2)
                .transition(() -> gamepad2.cross, TeleStates.INTAKE_2)

                // float
                .state(TeleStates.INTAKE_2)
                .onEnter(() -> {
                    r.intakeArm.setArm(IntakeArm.FLOAT_ARM);
                    r.intakeArm.release();
                    r.intakeArm.setSwivel(IntakeArm.SWIVEL_FLAT);
                })
                .loop(ic::update)
                .onExit(() -> {
                    ic.armUp = true;
                })
                .transition(()-> gamepad2.cross)

                .state(TeleStates.INTAKING_MACHINE)
                .onEnter(() -> {
                    intakingMachine.start();
                    ic.cancelArm = true;
                })
                .loop(() -> {
                    intakingMachine.update();
                    ic.update();
                })
                .onExit(() -> {
                    intakingMachine.reset();
                    intakingMachine.stop();
                    ic.armUp = true;
                    ic.cancelArm = false;
                })
                .transition(() -> intakingMachine.getState() == StateMachines.IntakingStates.FINAL && gamepad2.cross, TeleStates.TRANSFER)
                .transition(() -> intakingMachine.getState() == StateMachines.IntakingStates.FINAL && gamepad2.triangle, TeleStates.INTAKE_2)

                .state(TeleStates.TRANSFER)
                .onEnter(() -> {
                    transferMachine.start();
                    ic.resetToggle();
                })
                .loop(transferMachine::update)
                .transition(() -> (transferMachine.getState() == StateMachines.TransferStates.FINISHED) && (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.touchpad), () ->{
                })
                .transition(() -> gamepad2.triangle, TeleStates.INTAKE_2, () ->{
//                    r.intakeSlides.setPosition(IntakeSlides.PARTIAL);
                    r.outtake.toInit();
                })
                .onExit(() -> {
                    updateButtonMap();
                    telemetry.addData("butotn map bad", buttonMap);
                    transferMachine.stop();
                    transferMachine.reset();
                    oc.grabToggle = true;
                })

                .state(TeleStates.OUTTAKE)
                .onEnter(() -> {
//                    oc.transferButtonMap(buttonMap);
                    oc.grabToggle = true;
                    r.dr4b.setPosition(DR4B.OBSERVATION);
                    oc.outtakeState = OuttakeControl.OuttakeStates.OBSERVATION;
                })
                .loop(() -> {
                    r.outtake.grab();
                    oc.update();
                    drbc.update();
                })
                .transition(() -> gamepad2.square && !drbc.outtakeOnRung, TeleStates.DROP)
                .transition(() -> gamepad2.square && drbc.outtakeOnRung, TeleStates.COLLAPSE)
                .onExit(() -> {

                })

                .state(TeleStates.DROP)
                .onEnter(r.outtake::release)
                .transitionTimed(DROP_DELAY, TeleStates.NEUTRAL)

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
            telemetry.addData("button Map:", buttonMap);
            telemetry.addData("Tele State", machine.getState());
            telemetry.addData("Intake slide SET spositoin", r.intakeSlides.position);
            telemetry.addData("Intake slide curr position", r.intakeSlides.getRealPosition());

            machine.update();
            dc.update();
            r.update();
            telemetry.update();
        }
    }

    public void updateButtonMap() {
        buttonMap = new HashMap<Boolean, String>();
        buttonMap.put(gamepad1.dpad_up, "gamepad2.dpad_up");
        buttonMap.put(gamepad1.dpad_down, "gamepad2.dpad_down");
        buttonMap.put(gamepad1.dpad_left, "gamepad2.dpad_left");
        buttonMap.put(gamepad2.dpad_right, "gamepad2.dpad_right");
        buttonMap.put(gamepad1.touchpad, "gamepad1.touchpad");
        telemetry.addData("UPDATE DTHE BUTTOM MAP", buttonMap);
    }
}
