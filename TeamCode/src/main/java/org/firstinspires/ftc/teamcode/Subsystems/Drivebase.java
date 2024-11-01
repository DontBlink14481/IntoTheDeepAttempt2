package org.firstinspires.ftc.teamcode.Subsystems;

import

        com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;


@Config
public class Drivebase implements Subsystem {
    public Follower drive;
    public static final double CENTER_DIST = 8.25;
    public boolean driveDisable = false;

    public Drivebase(HardwareMap map, Pose pos) {
       drive = new Follower(map);
       drive.setPose(pos);
    }

    public Drivebase(HardwareMap map) {
        this(map, new Pose(0, 0, 0));
    }

    public void setPose(Pose pose) {
        drive.setPose(pose);
    }

    public void setPower(Pose powers) {
        drive.setTeleOpMovementVectors(powers.getX(), powers.getY(), powers.getHeading());
    }

    public Follower getDrive() {
        return drive;
    }

    public void followPath(Path path){
        drive.followPath(path);
    }

    @Override
    public void update(){
        drive.update();
    }

    public void toInit(){}
}
