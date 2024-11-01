package org.firstinspires.ftc.teamcode.Util;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;

public class NumericalObjectFilter {
    ArrayList<Filter> filters = new ArrayList<>();
    Class<?> c;
    public NumericalObjectFilter(Class<?> c, List<Complex> zeroes, List<Complex> poles, double gain){
        this.c = c;

        for(Field f : c.getFields()){
            if(!Modifier.isStatic(f.getModifiers())){
                filters.add(new Filter(zeroes, poles, gain));
            }
        }
    }

    public Object get(double input){
        try{
            Object o = c.getConstructor().newInstance();
            int index = 0;
            for (Field f : c.getFields()) {
                if (!Modifier.isStatic(f.getModifiers())) {
                    f.set(o, filters.get(index++).addInput(input));
                }
            }
            return o;
        }
        catch(Exception e){
            e.printStackTrace();
            return null;
        }

    }
}
