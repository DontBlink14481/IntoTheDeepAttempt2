package org.firstinspires.ftc.teamcode.TeleControl;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.FallingEdge;

@Config
public class OuttakeControl implements Control {

    Outtake outtake;
    Robot robot;
    FallingEdge swivelR = new FallingEdge(() -> outtake.swivelPos--);
    public static double speed = 0.01;
    FallingEdge swivelL = new FallingEdge(() -> outtake.swivelPos++);
    Gamepad gp1;
    Gamepad gp2;

    FallingEdge armBasket = new FallingEdge(() -> outtake.setArm(Outtake.ARM_BASKET));
    FallingEdge armSpecimen = new FallingEdge(() -> outtake.setArm(Outtake.ARM_SPECIMEN));

    FallingEdge wristBasket = new FallingEdge(() -> outtake.setWrist(Outtake.WRIST_BASKET));
    FallingEdge wristSpecimen = new FallingEdge(() -> outtake.setWrist(Outtake.WRIST_SPECIMEN));



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
        if(gp2.right_stick_x *gp2.right_stick_x + gp2.right_stick_y * gp2.right_stick_y > (deadzone * deadzone)){
            outtake.setSwivel(Outtake.joystickToSwivel(gp2.right_stick_x, gp2.right_stick_y));
        }
        else outtake.setSwivel(Outtake.SWIVEL_OUTTAKE);

        armSpecimen.update(gp2.dpad_up || gp2.dpad_down);
        wristSpecimen.update(gp2.dpad_up || gp2.dpad_down);

        armBasket.update(gp2.dpad_left || gp2.left_bumper);
        wristBasket.update(gp2.dpad_left || gp2.left_bumper);

    }

    @Override
    public void addTelemetry(Telemetry telemetry) {

    }

}
