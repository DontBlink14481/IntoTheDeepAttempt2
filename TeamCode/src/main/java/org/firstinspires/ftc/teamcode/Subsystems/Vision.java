package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.AprilTagLocalizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Vision implements Subsystem{

    public VisionPortal vp;
    public VisionPortal.Builder vpBuilder;
    public List<AprilTagDetection> detections;
    public List<VisionProcessor> processors;
    HardwareMap hardwareMap;
    Telemetry t;
    public Pose unfilteredVisionPose = null;
    private boolean camInited = false;

    public Vision(HardwareMap map, Telemetry t, VisionProcessor... v) {
        this.t = t;
        hardwareMap = map;
        vpBuilder = vpBuilder();
        processors = (v == null) ? new ArrayList<>() : Arrays.asList(v);
    }

    @Override
    public void update() {
        for(VisionProcessor proc : processors){
            if(proc instanceof AprilTagProcessor){
                detections = ((AprilTagProcessor)proc).getDetections();
                if(detections == null) continue;

                t.addData("over 1 detection", true);

                int count = 0;
                double xAverage = 0, yAverage = 0, headingAverage = 0;
                for(AprilTagDetection detection : detections){
                    if((detection.metadata != null)){
                        Pose position = AprilTagLocalizer.finalShifted(detection);
                        xAverage += position.getX();
                        yAverage += position.getY();
                        headingAverage += position.getHeading();
                        count++;
                    }
                }

                t.addData("Count", count);


                if(count > 0) unfilteredVisionPose = new Pose(xAverage/count, yAverage/count, headingAverage/count);
                else unfilteredVisionPose = null;
            }
        }

    }


    //build
    @Override
    public void toInit() {
        if(!camInited){
            initProcessors();
            vp = vpBuilder.build();

            vp.resumeStreaming();
            camInited = true;
        }
    }

    private VisionPortal.Builder vpBuilder() {

        // Create a VisionPortal builder
        return new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

    }

    private void initProcessors() {
        if(camInited) return;
        for (VisionProcessor processor : processors) {
            vpBuilder.addProcessor(processor);
        }
    }

    public void addAprilTag(){
        addProcessors(new AprilTagProcessor.Builder().setDrawAxes(true).build());
    }

    private void addProcessors(VisionProcessor... processors) {
        if(camInited) return;
        for (VisionProcessor processor : processors) {
            vpBuilder.addProcessor(processor);
        }
    }
}
