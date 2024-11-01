package org.firstinspires.ftc.teamcode.Util;

import android.util.Pair;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;

public class VariableScraper {
    private final Class<?> classObj;
    private Object obj;
    private String name;
    private Log l;
    private boolean showStatic = true;

    //static vars
    public VariableScraper(Class<?> c, boolean log) {
        classObj = c;
        obj = null;
        if (log) open();
    }


    //all vars
    public <T> VariableScraper(T o, boolean log, String name, boolean ss) {
        classObj = o.getClass();
        obj = o;
        this.name = name;
        showStatic = ss;
        if (log) open();
    }

    public static ArrayList<Pair<String, String>> getAllVariables(Class<?> classObj, Object obj, boolean showStatic) {
        ArrayList<Pair<String, String>> output = new ArrayList<>();
        for (Field f : classObj.getDeclaredFields()) {
            if (Modifier.isPublic(f.getModifiers())) {
                if (Modifier.isStatic(f.getModifiers()) && !showStatic) continue;
                String type = f.getType().getSimpleName();
                String name = f.getName();
                String value;
                try {
                    value = (f.get(obj)).toString();
                } catch (IllegalAccessException e) {
                    value = "Illegal Access";
                } catch (NullPointerException e) {
                    value = "null";
                } catch (IllegalArgumentException e) {
                    value = "Illegal Argument";
                } catch (Exception e) {
                    value = "Unknown Error";
                    e.printStackTrace();
                }
                output.add(new Pair<String, String>(type + " " + name, value));

            }
        }
        return output;
    }

    public ArrayList<Pair<String, String>> getAllVariables() {
        return getAllVariables(classObj, obj, showStatic);
    }

    public void writeVariablesToTelemetry(Telemetry t) {
        ArrayList<Pair<String, String>> vars = getAllVariables();
        if (obj == null) t.addData("Class", classObj.getSimpleName());
        else t.addData("Variable " + name, classObj.getSimpleName());
        for (Pair<String, String> p : vars) {
            t.addData(p.first, p.second);
        }
        t.addData("\n", "\n");
    }

    public void writeVariablesToLog() {
        if (l == null) return;
        ArrayList<Pair<String, String>> vars = getAllVariables();
        l.addData("(");
        for (int i = 0; i < vars.size(); i++) {
            l.addData(vars.get(i).second + (i == vars.size()-1 ? "" : ","));
        }

        // original
        /*for (Pair<String, String> p : vars) {
//            l.addData(p.first + ": " + p.second + "\n");
            l.addData(p.second + ",");
        }*/
        l.addData(")");
        l.addData("\n");
        l.update();
    }

//    public void update() {
//        if (l != null) l.update();
//    }

    public void close() {
        if (l != null) l.close();
    }

    public void open() {
        if (obj == null) {
            l = new Log(classObj.getSimpleName() + "_values", true, "txt");
        } else {
            l = new Log(classObj.getSimpleName() + "_" + name + "_values", true, "txt");

        }
    }
}
