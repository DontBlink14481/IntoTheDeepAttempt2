package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Util.NumericalObjectInterpLUT;
import org.firstinspires.ftc.teamcode.Util.Util;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Robot {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public DR4B dr4b;

    public double error = -1;

    public Drivebase drive;
    public Outtake outtake;
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public static VoltageSensor voltageSensor;
    public Vision vision;

    // making a list of subsystem
    public List<Subsystem> subsystems;
    public double modifiedCount = 0;
    public static NumericalObjectInterpLUT outtakeRowLUT;
    public static NumericalObjectInterpLUT outtakeColLUT;

    public static double DRIVE_OFFSET = -3;

    // initializing other subsystems

    public Robot(HardwareMap map, Telemetry t, Pose start, boolean resetSlidesEncoder){
        hardwareMap = map;
        telemetry = t;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        drive = new Drivebase(hardwareMap, start);
        outtake = new Outtake(hardwareMap);
        dr4b = new DR4B(hardwareMap);
        intakeArm = new IntakeArm(hardwareMap);
        intakeSlides = new IntakeSlides(hardwareMap, resetSlidesEncoder);
        subsystems = new ArrayList<>(Arrays.asList(dr4b, drive, outtake, intakeSlides, intakeArm));

    }

    public Robot(HardwareMap map, Telemetry t,  Pose start) {
        this(map, t, start, false);
    }



    public Robot(HardwareMap map, Telemetry telemetry) {
        this(map, telemetry, Util.getPoseFromFile());
    }

    public void activateVision(VisionProcessor... visionProcessors){
        vision = new Vision(hardwareMap, telemetry, visionProcessors);
        subsystems.add(vision);
    }

    public VectorF getCameraPos() {
        Vector cameraPos2d = drive.drive.getPose().getHeadingVector().times(-drive.CENTER_DIST).plus(drive.drive.getPose().getVector());
        return new VectorF((float) cameraPos2d.getXComponent(), (float) cameraPos2d.getYComponent(), 5.0f);
    }

    public Orientation getCameraOrientation() {
        return new Orientation(AxesReference.EXTRINSIC, AxesOrder.YZX,
                AngleUnit.RADIANS, 0, (float) drive.drive.getPose().getHeading(), 0f, 0);
    }



    public void update() {
        for (Subsystem s : subsystems) {
            s.update();
        }
        if(vision != null){

        }
        Util.writePosToFile(this);
    }


    public void toInit() {
        for (Subsystem s : subsystems) {
            s.toInit();
        }
    }
}
