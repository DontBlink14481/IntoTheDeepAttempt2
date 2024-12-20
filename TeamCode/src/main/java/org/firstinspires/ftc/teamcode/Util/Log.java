package org.firstinspires.ftc.teamcode.Util;

import android.os.Environment;

import android.util.Pair;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

/**
 * Created by Gabriel on 2018-01-03.
 * A simple way to log data to a file.
 * 🫡🫡🫡
 */

//TODO: Make it implement Telemetry
public class Log {
    private static final String BASE_FOLDER_NAME = "FIRST";
    private Writer fileWriter;
    private String line;
    private boolean logTime;
    private long startTime;
    private boolean disabled = false;
    public String fullFileName;
    public Log(String filename, boolean logTime) {
        new Log(filename, logTime, "csv");
    }

    public Log(String filename, boolean logTime, String fileExtension){
        if (logTime) startTime = System.nanoTime();
        this.logTime = logTime;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        //noinspection ResultOfMethodCallIgnored
        directory.mkdir();
        fullFileName = filename + "." + fileExtension;
        try {
            fileWriter = new FileWriter(directoryPath+"/"+filename+"." + fileExtension);
        } catch (IOException e) {
            e.printStackTrace();
        }
        line = "";
    }


    // use only for specific variables otherwise use Util.getAllVariables()
    @SafeVarargs
    public final void addVariables(Pair<String, Object>... trackers) {
        for(Pair<String, Object> obj : trackers) {
            addData(obj.first, obj.second);
        }
    }

    public boolean isDisabled() {
        return disabled;
    }

    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }

    public void close() {
        try {
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void update() {
        if (disabled) return;
        try {
            if (logTime) {
                long timeDifference = System.nanoTime()-startTime;
                line = timeDifference / 1E9 + "\n" + line;
            }
            fileWriter.write(line+"\n");
            line = "";
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static File pullFile(String filename, String extension){
        return new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + filename + "." + extension);
    }


    //ours
    public void addData(Vector v){
        if (disabled) return;
        line += v.getXComponent() + "," + v.getYComponent() + "\n";
    }

    public void addData(String data, Object stuff) {
        if (disabled) return;
        if (!line.isEmpty()) line += (data +  ": " + stuff.toString());
        line += data;
    }

    //theirs
    public void addData(String data) {
        if (disabled) return;
//        if (!line.equals("")) line += ",";
        line += data;
    }



    public void addData(Object data) {
        addData(data.toString());
    }
    public void addData(boolean data) {
        addData(String.valueOf(data));
    }
    public void addData(byte data) {
        addData(String.valueOf(data));
    }
    public void addData(char data) {
        addData(String.valueOf(data));
    }
    public void addData(short data) {
        addData(String.valueOf(data));
    }
    public void addData(int data) {
        addData(String.valueOf(data));
    }
    public void addData(long data) {
        addData(String.valueOf(data));
    }
    public void addData(float data) {
        addData(String.valueOf(data));
    }
    public void addData(double data) {
        addData(String.valueOf(data));
    }
}