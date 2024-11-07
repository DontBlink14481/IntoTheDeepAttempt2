package org.firstinspires.ftc.teamcode.Util;

import android.view.CollapsibleActionView;

import com.acmerobotics.dashboard.config.Config;
import com.sfdev.assembly.state.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DR4B;
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

    public enum CollapseStates{
        CLIP,
        RELEASE,
        DROP,
        COLLAPSE,
        FINISHED
    }
    //transfer constants
    public static double INTAKE_BREAK = 1;

    //collapse constants
    public static double RELEASE_TIME = 0.2;
    public static double TRAVEL_TIME = 0.5;


    public static StateMachine getTransferMachine(Robot robot, Telemetry telemetry) {
        IntakeArm intakeArm = robot.intakeArm;
        Outtake outtake = robot.outtake;
        IntakeSlides slides = robot.intakeSlides;
        telemetry.addData("went to transfer machine", ".");

        // to init being transfer positions
        return new StateMachineBuilder()
                .state(TransferStates.PRE_TRANSFER)
                .onEnter(() -> {
                    robot.toInit();
                    outtake.release();
                })
                .transition(() -> robot.intakeSlides.getRealPosition() < IntakeSlides.admissible)

                .state(TransferStates.INTAKE_BREAK)
                .transitionTimed(INTAKE_BREAK)
                .onExit(outtake::grab)

                .state(TransferStates.FINISHED)


                .build();
    }

    public static StateMachine getCollapseMachine(Robot r, Telemetry t){
        return new StateMachineBuilder()
                .state(CollapseStates.CLIP)
                .onEnter(() -> r.outtake.outtakeInter())
                .transition(() -> r.outtake.armSetPosition == -1)

                .state(CollapseStates.RELEASE)
                .onEnter(r.outtake::release)
                .transitionTimed(RELEASE_TIME)

                .state(CollapseStates.COLLAPSE)
                .onEnter(() -> {
                    r.dr4b.setPosition(DR4B.BASE);
                    r.outtake.transfer();
                })
                .transition(() -> Util.isCloseEnough(r.dr4b.getAngle(), r.dr4b.getPosition(), DR4B.acceptable))
                .state(CollapseStates.FINISHED)
                .build();
    }


}
