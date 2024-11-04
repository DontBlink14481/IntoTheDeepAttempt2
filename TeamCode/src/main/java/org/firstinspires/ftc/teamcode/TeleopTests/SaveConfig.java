package org.firstinspires.ftc.teamcode.TeleopTests;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Log;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;

@Config
@TeleOp
public class SaveConfig extends LinearOpMode {
    public static String prefix = Environment.getExternalStorageDirectory().getPath() + "/FIRST/";
    public static String filename = RobotConfigFileManager.stripFileNameExtension(new RobotConfigFileManager().getActiveConfig().getName());
    public static String oldFile = prefix + filename + ".xml";
    public static String newFile = prefix + filename + ".txt";
    public boolean wrote = false;
    public IOException i;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            Scanner sc = new Scanner(new File(oldFile));
            Log l = new Log(filename, false, "txt");
            while (sc.hasNext()) {
                String s = sc.nextLine();
                l.addData(s + "\n");
                telemetry.addData("L", s);
            }
            sc.close();
            l.update();
            l.close();
            wrote = true;
        } catch (IOException e) {
            e.printStackTrace();
            i = e;
        }
        telemetry.update();

        waitForStart();
        MultipleTelemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        while (opModeIsActive()) {
            t.addData("Old Path", oldFile);
            t.addData("New Path", newFile);
            t.addData("written", (wrote) ? "yep" : "nope");
            if (!wrote) t.addData("error", i.toString());
            t.update();

        }
    }
}
