package org.firstinspires.ftc.teamcode.TeleControl;

import static org.firstinspires.ftc.teamcode.TeleControl.OuttakeControl.deadzone;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.FallingEdge;

@Config
public class IntakeControl implements Control {

    IntakeSlides slides;
    IntakeArm arm;
    Robot robot;
    Gamepad gp1, gp2;
    public boolean armUp = false;
    public static double slidesSpeed = 150;
    public boolean cancelArm = false;
    FallingEdge grab = new FallingEdge(() -> arm.grab());
    FallingEdge swapArm = new FallingEdge(() -> armUp = !armUp);
    FallingEdge flat = new FallingEdge(() -> arm.setSwivel(IntakeArm.SWIVEL_FLAT));
    FallingEdge swivelLeft = new FallingEdge(() -> arm.setSwivel(IntakeArm.SWIVEL_LEFT));
    FallingEdge swivelRight = new FallingEdge(() -> arm.setSwivel(IntakeArm.SWIVEL_RIGHT));

    FallingEdge clawSemiOpen = new FallingEdge(() -> arm.setClaw(IntakeArm.CLAW_SEMI));

    FallingEdge slidesIn = new FallingEdge(() -> {
        slides.setPosition(IntakeSlides.IN);
        slidesPartialToggle = true;
    });

    private boolean slidesPartialToggle = true;
    FallingEdge extendToggle = new FallingEdge(() -> {
        slides.setPosition(slidesPartialToggle ? IntakeSlides.PARTIAL : IntakeSlides.EXTENDED);
        slidesPartialToggle = !slidesPartialToggle;
    } );

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

//        if (gp2.right_trigger == 0 && gp2.left_trigger == 0) {
//            arm.setSpinner(0);
//        } else if (gp2.left_trigger != 0) {
//            arm.setSpinner(0.5 * gp2.left_trigger);
//        } else if(gp2.right_trigger != 0) {
//            arm.setSpinner(-gp2.right_trig        ger);
//        }

        if(gp2.left_stick_y != 0) {
            slides.setPosition(slides.position - slidesSpeed * gp2.left_stick_y);
//            slides.motionProfile = false;
        }
        else{
//            slides.motionProfile = true;
        }
        slidesIn.update(gp2.left_bumper);
        clawSemiOpen.update(gp1.circle);

        extendToggle.update(gp2.right_bumper);

        swapArm.update(gp2.circle);

        if(!cancelArm) arm.setArm(armUp ? IntakeArm.FLOAT_ARM : IntakeArm.ARM_BUMP);

        flat.update(gp2.touchpad);
        swivelLeft.update(gp2.left_trigger > 0.1);
        swivelRight.update(gp2.right_trigger > 0.1);

        /*if(gp2.right_stick_x *gp2.right_stick_x + gp2.right_stick_y * gp2.right_stick_y > (deadzone * deadzone)){
            arm.setSwivel(Range.clip(IntakeArm.joystickToSwivel(gp2.right_stick_x, -gp2.right_stick_y), 0, 1));
        }
        else arm.setSwivel(IntakeArm.SWIVEL_FLAT);*/


    }

    @Override
    public void addTelemetry(Telemetry telemetry) {

    }

    public void resetToggle() {
        slidesPartialToggle = true;
    }


}
