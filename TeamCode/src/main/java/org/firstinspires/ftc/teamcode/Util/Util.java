package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

import java.io.IOException;
import java.util.List;
import java.util.Scanner;


public class Util {
    public static String autoFileName = "autoPos";
    public static double ROBOT_RADIUS = 9;
    private Util(){}

    public static boolean isCloseEnough(Pose p1, Pose p2, double error, double hError){
        return isCloseEnough(p1.getX(), p2.getX(), error)
                && isCloseEnough(p1.getX(), p2.getY(), error)
                && (isCloseEnough(p1.getHeading(), p2.getHeading(), hError)
                || isCloseEnough((Math.max(p1.getHeading(), p2.getHeading())-Math.toRadians(360)),
                    Math.min(p1.getHeading(), p2.getHeading()), hError));
    }


    public static boolean isCloseEnough(double v1, double v2, double error){
        return Math.abs(v1-v2) <= error;
    }
    public static boolean headingIsCloseEnough(double v1, double v2, double error){
        while(v1 - v2 > Math.PI){
            v1 += 2 * Math.PI;
        }
        while(v1 - v2 < -Math.PI){
            v2 += 2 * Math.PI;
        }
        return isCloseEnough(v1, v2, error);
    }

    public static Object nary(List<Boolean> conds, List<Object> outputs){
        for(int i = 0; i < conds.size(); i++){
            if(conds.get(i)) return outputs.get(i);
        }
        return outputs.get(outputs.size() - 1);
    }

    public static void writePosToFile(Robot robot){
        Pose pos = robot.drive.getDrive().getPose();
        Log l = new Log(autoFileName, false, "txt");
        robot.telemetry.addData("writing pose", pos.toString());
        l.addData(pos.getX() + "\n");
        l.addData(pos.getY() + "\n");
        l.addData(pos.getHeading());
        l.update();
        l.close();
    }

    public static Pose getPoseFromFile(){
        try{
            Scanner sc = new Scanner(Log.pullFile(autoFileName, "txt"));
            if(!sc.hasNext()) throw new IOException();
            return new Pose(sc.nextDouble(), sc.nextDouble(), sc.nextDouble());
        }
        catch(Exception e){
            e.printStackTrace();
            return new Pose(0, 0, Math.PI);
        }

    }



    public static void pushPoseToDash(Pose... poses){
        TelemetryPacket tPack = new TelemetryPacket();
        Canvas c = tPack.fieldOverlay();
        for(int i = 0; i < poses.length; i++){
            if(poses[i] == null) continue;
            c.setStroke(getDrawColor(i));
            drawRobot(c, poses[i]);
        }

        FtcDashboard.getInstance().sendTelemetryPacket(tPack);

    }

    public static void drawRobot(Canvas canvas, Pose pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        Vector v = pose.getHeadingVector().times(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public static String getDrawColor(int num){
        switch (num){
            case 0:
                return "#3F51B5";
            default:
                return "#46F743";
        }
    }


}
