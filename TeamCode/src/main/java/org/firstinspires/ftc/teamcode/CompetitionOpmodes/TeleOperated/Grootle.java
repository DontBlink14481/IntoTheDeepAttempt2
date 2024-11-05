package org.firstinspires.ftc.teamcode.CompetitionOpmodes.TeleOperated;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.StateMachines;

// our controller operated mode


@TeleOp
@Config
public class Grootle extends LinearOpMode {

    public enum TeleStates {
        NEUTRAL, INTAKE, TRANSFER, OUTTAKE

    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot r = new Robot(hardwareMap, telemetry);

        StateMachine transferMachine = StateMachines.getTransferMachine(r, telemetry);


        StateMachine tele = new StateMachineBuilder()
                .state(TeleStates.NEUTRAL)
                .onEnter(() -> {
                    r.toInit();
                })
                .transition(() -> gamepad2.right_trigger > 0.1, TeleStates.INTAKE)

                .state(TeleStates.INTAKE)
                .onEnter(() -> r.intakeSlides.setPosition(IntakeSlides.PARTIAL))
                .transition(() -> gamepad2.x, TeleStates.TRANSFER)

                .state(TeleStates.TRANSFER)
                .onEnter(transferMachine::start)
                .loop(transferMachine::update)
                .transition(() -> !transferMachine.isRunning() && gamepad2.square)
                .onExit(() -> {
                    transferMachine.stop();
                    transferMachine.reset();
                })

                .state(TeleStates.OUTTAKE)
                .onEnter(() -> {
//                    r.outtake.set
                })

                .build();

        waitForStart();
        tele.start();
        while (opModeIsActive()) {
            tele.update();

            r.update();
        }
    }
}
