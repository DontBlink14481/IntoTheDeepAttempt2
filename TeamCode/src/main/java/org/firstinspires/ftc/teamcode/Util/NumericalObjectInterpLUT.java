package org.firstinspires.ftc.teamcode.Util;

import com.arcrobotics.ftclib.util.InterpLUT;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;

public class NumericalObjectInterpLUT {
    ArrayList<InterpLUT> interpluts = new ArrayList<>();
    Class<?> c;
    public NumericalObjectInterpLUT(Class<?> c){
        this.c = c;

        for(Field f : c.getFields()){
            if(!Modifier.isStatic(f.getModifiers())){
                interpluts.add(new InterpLUT());
            }
        }
    }

    public void add(Double input, Object o){
        try {
            if (!(c.isInstance(o))) return;
            int index = 0;

            for (Field f : c.getFields()) {
                if (!Modifier.isStatic(f.getModifiers())) {
                    Number value = (Number) (f.get(o));
                    interpluts.get(index++).add(input, value.doubleValue());
                }
            }
        }
        catch (Exception e){
            e.printStackTrace();
        }
    }

    public void build(){
        for(InterpLUT i : interpluts){
            i.createLUT();
        }
    }

    public Object get(double input){
         try{
            Object o = c.getConstructor().newInstance();
            int index = 0;
            for (Field f : c.getFields()) {
                if (!Modifier.isStatic(f.getModifiers())) {
                    f.set(o, interpluts.get(index++).get(input));
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
