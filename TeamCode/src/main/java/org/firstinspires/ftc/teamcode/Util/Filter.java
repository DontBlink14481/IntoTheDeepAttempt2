package org.firstinspires.ftc.teamcode.Util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class Filter {
    Polynomial numerator;
    Polynomial denominator;
    List<Double> prevInputs;
    List<Double> prevOutputs;
    List<Double> xCoeffs;
    List<Double> yCoeffs;
    double gain;

    public Filter(List<Complex> zeroes, List<Complex> poles, double gain){
        if(poles == null){
            poles = new ArrayList<>();
        }

        this.gain = gain;
        ArrayList<Complex> conj = new ArrayList<>();
        for(Complex zero : zeroes){
            if(zero.imag() != 0){
                conj.add(zero.conjugate());
            }
        }
        zeroes.addAll(conj);

        conj = new ArrayList<>();
        for(Complex pole : poles){
            if(pole.imag() != 0){
                conj.add(pole.conjugate());
            }
        }
        poles.addAll(conj);
        while(poles.size() < zeroes.size()){
            poles.add(Complex.ORIGIN);
        }


        denominator = Polynomial.fromRoots(poles);
        numerator = Polynomial.fromRoots(zeroes);
        prevInputs = new ArrayList<>();
        prevOutputs = new ArrayList<>();
    }


    public List<Double> getXDiffCoeffs(){
        if(xCoeffs != null) return xCoeffs;
        ArrayList<Double> out = new ArrayList<>();
        Complex[] terms = numerator.getTerms();
        for(Complex term : terms){
            out.add(0, term.real());
        }

        while(out.size() < denominator.degree() + 1){
            out.add(0,0.0);
        }
        xCoeffs = out;
        return out;
    }

    public List<Double> getYDiffCoeffs(){
        if(yCoeffs != null) return yCoeffs;
        ArrayList<Double> out = new ArrayList<>();
        Complex[] terms = denominator.getTerms();
        for(int i = terms.length - 2; i >= 0; i--){
            out.add(-terms[i].real());
        }
        yCoeffs = out;
        return out;
    }

    public double addInput(double input){
        prevInputs.add(0, input);
        while(prevInputs.size() > numerator.degree() + 1){
            prevInputs.remove(prevInputs.size() - 1);
        }

        double output = 0;

        for(int i = 0; i < prevInputs.size(); i++) {
            output += gain * xCoeffs.get(i) * prevInputs.get(i);
        }

        for(int i = 0; i < prevOutputs.size(); i++) {
            output += yCoeffs.get(i) * prevOutputs.get(i);
        }

        prevOutputs.add(0, output);
        while(prevInputs.size() > denominator.degree()){
            prevInputs.remove(prevInputs.size() - 1);
        }

        return output;
    }

    @Override
    public String toString(){
        return "Discrete transfer function: (" + numerator.toString() + ")/("+denominator.toString() + ")" +
                "\nX Coeffs: " + xCoeffs.toString() +
                "\nYCoeffs" + yCoeffs.toString() +
                "\nPrev Inputs" + prevInputs.toString() +
                "\nPrev Outputs" + prevOutputs.toString();
    }
}
