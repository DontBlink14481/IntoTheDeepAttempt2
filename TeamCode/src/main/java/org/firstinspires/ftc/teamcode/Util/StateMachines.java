package org.firstinspires.ftc.teamcode.Util;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Config
public class StateMachines {
    public enum Transfer {
        PRE_TRANSFER
    }

    public double INTAKE_BREAK = 0.2;

    public static StateMachine getTransferMachine(Robot robot, Telemetry telemetry) {
        IntakeArm intakeArm = new IntakeArm(robot, telemetry);
        Outtake outtake = robot.outtake;
        IntakeSlides slides = robot.intakeSlides;
        telemetry.addData("went to transfer machine", ".");

        return new StateMachineBuilder()
                .state(Transfer.PRE_TRANSFER)
                .onEnter(() -> {
                    // to init being transfer positions
                    robot.toInit();
                })
                .transition(() -> robot.intakeSlides.getPosition() < IntakeSlides.admissible)

                .state(Transfer.INTAKE_BREAK)
                .transitionTimed(INTAKE_BREAK)


                .build();
    }
}
