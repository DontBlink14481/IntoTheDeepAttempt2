package org.firstinspires.ftc.teamcode.TeleControl;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.FallingEdge;

@Config
public class IntakeControl implements Control {

    IntakeSlides slides;
    IntakeArm arm;
    Robot robot;
    Gamepad gp1, gp2;
    public boolean armUp = true;
    public static double slidesSpeed = 1;

    FallingEdge grab = new FallingEdge(() -> arm.grab());
    FallingEdge swapArm = new FallingEdge(() -> armUp = !armUp);

    public IntakeControl(IntakeSlides slides, IntakeArm arm, Gamepad gp1, Gamepad gp2) {
        this.slides = slides;
        this.arm = arm;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public IntakeControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.intakeSlides, robot.intakeArm, gp1, gp2);
        this.robot = robot;
    }



    @Override
    public void update() {
        arm.setArm(armUp ? IntakeArm.FLOAT_ARM : IntakeArm.ARM_GRAB);

        swapArm.update(gp2.circle);
        grab.updateOnPress(gp2.cross);

//        if (gp2.right_trigger == 0 && gp2.left_trigger == 0) {
//            arm.setSpinner(0);
//        } else if (gp2.left_trigger != 0) {
//            arm.setSpinner(0.5 * gp2.left_trigger);
//        } else if(gp2.right_trigger != 0) {
//            arm.setSpinner(-gp2.right_trigger);
//        }

//        slides.setPosition(slides.getRealPosition() - slidesSpeed * gp2.left_stick_y);

        if (gp2.right_trigger > 0.1) {
            slides.setPosition(IntakeSlides.PARTIAL);
        }

        if (gp2.left_trigger > 0.1) {
            slides.setPosition(IntakeSlides.IN);
        }



    }

    @Override
    public void addTelemetry(Telemetry telemetry) {

    }


}
