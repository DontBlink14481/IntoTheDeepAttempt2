package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.sfdev.assembly.state.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Config
public class StateMachines {
    public enum TransferStates {
        PRE_TRANSFER,
        INTAKE_BREAK, FINISHED
    }

    public enum OuttakeStates {
        PRE_OUTTAKE,
        INTAKE_BREAK, FINISHED
    }

    public static double INTAKE_BREAK = 0.2;

    public static StateMachine getTransferMachine(Robot robot, Telemetry telemetry) {
        IntakeArm intakeArm = robot.intakeArm;
        Outtake outtake = robot.outtake;
        IntakeSlides slides = robot.intakeSlides;
        telemetry.addData("went to transfer machine", ".");

        // to init being transfer positions
        return new StateMachineBuilder()
                .state(TransferStates.PRE_TRANSFER)
                .onEnter(robot::toInit)
                .transition(() -> robot.intakeSlides.getRealPosition() < IntakeSlides.admissible)

                .state(TransferStates.INTAKE_BREAK)
                .transitionTimed(INTAKE_BREAK)
                .onExit(outtake::grab)

                .state(TransferStates.FINISHED)


                .build();
    }


}
