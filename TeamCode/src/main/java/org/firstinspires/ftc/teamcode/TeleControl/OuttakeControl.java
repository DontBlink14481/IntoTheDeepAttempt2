package org.firstinspires.ftc.teamcode.TeleControl;

import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.ARM_BASKET;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.ARM_OBSERVATION;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.ARM_SPECIMEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.WRIST_BASKET;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.WRIST_OBSERVATION;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.WRIST_SPECIMEN;
import static org.firstinspires.ftc.teamcode.TeleControl.OuttakeControl.OuttakeStates.BASKET;
import static org.firstinspires.ftc.teamcode.TeleControl.OuttakeControl.OuttakeStates.OBSERVATION;
import static org.firstinspires.ftc.teamcode.TeleControl.OuttakeControl.OuttakeStates.RUNG;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.FallingEdge;

@Config
public class OuttakeControl implements Control {

    public enum OuttakeStates {
        OBSERVATION, RUNG, BASKET
    }

    Outtake outtake;
    Robot robot;
    public static double speed = 0.01;
    public boolean grabToggle = true;
    Gamepad gp1;
    Gamepad gp2;

    public OuttakeStates outtakeState = OuttakeStates.OBSERVATION;



    FallingEdge basket = new FallingEdge(() -> outtakeState = BASKET);
    FallingEdge specimen = new FallingEdge(() -> outtakeState = RUNG);
    FallingEdge observation = new FallingEdge(() -> outtakeState = OBSERVATION);

    FallingEdge toggleClaw = new FallingEdge(() -> grabToggle = !grabToggle);




    public static double deadzone = 0.2;


    public OuttakeControl(Outtake o, Gamepad gp1, Gamepad gp2) {
        this.outtake = o;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public OuttakeControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.outtake, gp1, gp2);
        this.robot = robot;
    }


    @Override
    public void update() {
//        if(gp2.right_stick_x *gp2.right_stick_x + gp2.right_stick_y * gp2.right_stick_y > (deadzone * deadzone)){
//            outtake.setSwivel(Outtake.joystickToSwivel(gp2.right_stick_x, gp2.right_stick_y));
//        }
//        else outtake.setSwivel(Outtake.SWIVEL_OUTTAKE);

        outtake.setSwivel(Outtake.SWIVEL_OUTTAKE);

//        outtake.setWrist(outtake.getGoodWristPosition(outtakeRung ? 0 : Outtake.BASKET_ANGLE));

        if(grabToggle){
            outtake.grab();
        }
        else outtake.release();

        switch(outtakeState){
            case OBSERVATION:
                outtake.setArm(ARM_OBSERVATION);
                outtake.setWrist(WRIST_OBSERVATION);
                break;
            case RUNG:
                outtake.setArm(ARM_SPECIMEN);
                outtake.setWrist(WRIST_SPECIMEN);
                break;
            case BASKET:
                outtake.setArm(ARM_BASKET);
                outtake.setWrist(WRIST_BASKET);
                break;
            default:
                break;
        }



        toggleClaw.update(gp2.square);

        specimen.updateOnPress(gp2.dpad_right || gp2.dpad_down);
        basket.updateOnPress(gp2.dpad_left || gp2.dpad_up);
        observation.updateOnPress(gp2.touchpad);
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {

    }

}
