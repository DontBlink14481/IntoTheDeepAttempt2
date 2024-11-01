package org.firstinspires.ftc.teamcode.Util;
import java.util.*;
public class Polynomial implements Cloneable, Comparable<Polynomial>{
    private Complex[] terms;
    // creates polynomial as the sum of terms[i]*x^i
    public Polynomial(Complex[] terms){
        this.terms = terms;
    }
    // creates polynomial with 10 terms
    public Polynomial() {
        terms = new Complex[10];
        Arrays.fill(terms, Complex.ORIGIN);
    }
    public static Polynomial fromRoots(List<Complex> roots){
        Polynomial ans = new Polynomial();
        ans.add(1,0);
        for(Complex c: roots) ans = ans.multiply(new Polynomial(new Complex[]{Complex.ONE,c.negate()}));
        return ans;
    }
    // converts string to polynomial, x can be lowercase or uppercase
    public Polynomial(String s) throws Exception{
        int p = 0;
        terms = new Complex[10];
        Arrays.fill(terms, Complex.ORIGIN);
        while(p < s.length()){
            int pp = p;
            boolean neg = false;
            Complex coeff2 = Complex.NaN;
            double coeff = 0;
            if(s.charAt(p) == '-'){
                neg = true;
                p++;
            }else if(s.charAt(p) == '+') p++;
            if(s.charAt(p) == '(') {
                int ppp = p;
                while(p < s.length() && s.charAt(p) != ')') p++;
                coeff2 = new Complex(s.substring(ppp+1, p));
                p++;
                if(neg) coeff2 = coeff2.negate();
            }else {
                if(s.charAt(p) == 'x' || s.charAt(p) == 'X') coeff = neg ? -1:1;
                else{
                    double num = 0;
                    boolean in = false;
                    double x = 0;
                    while(p < s.length() && (Character.isDigit(s.charAt(p)) || s.charAt(p) == '.')){
                        if(s.charAt(p) == '.'){
                            in = true;
                            p++;
                            continue;
                        }
                        if(in){
                            x*=10;
                            x+=s.charAt(p++)-'0';
                        }else{
                            num*=10;
                            num+=s.charAt(p++)-'0';
                        }
                    }
                    while(x >= 1) x/=10;
                    num+=x;
                    coeff = neg ? -num:num;
                }
            }
            int pow = 0;
            if(p < s.length() && (s.charAt(p) == 'x' || s.charAt(p) == 'X')){
                p++;
                pow = 1;
            }
            if(p < s.length() && s.charAt(p) == '^') p++;
            int num = 0;
            while(p < s.length() && Character.isDigit(s.charAt(p))){
                num*=10;
                num+=s.charAt(p++)-'0';
            }
            if(num != 0) pow = num;
            if(coeff2.isNaN()) add(coeff, pow);
            else add(coeff2, pow);
            if(p == pp) throw new IllegalArgumentException("Invalid Polynomial");
        }
    }
    // creates a polynomial with a degree
    public Polynomial(int degree){
        if(degree < 0) throw new IllegalArgumentException("Degree cannot be negative");
        terms = new Complex[degree+1];
        Arrays.fill(terms, Complex.ORIGIN);
    }
    // creates lowest degree polynomial that contains all points (X.get(i), Y.get(i))
    public Polynomial(List<Complex> X, List<Complex> Y) {
        if(X.size() != Y.size()) throw new IllegalArgumentException("Points don't match");
        for(int i = 0; i<X.size(); i++) for(int j = 0; j<i; j++) if(X.get(i).subtract(X.get(j)).radius() <= 1e-8)
            throw new IllegalArgumentException("Invalid function");
        List<List<Complex>> equations = new ArrayList<>(X.size());
        for(int i = 0; i<X.size(); i++) {
            equations.add(new ArrayList<>(X.size()+1));
            for(int j = X.size()-1; j>=0; j--) equations.get(i).add(X.get(i).pow(j));
            equations.get(i).add(Y.get(i));
        }
        int pow = X.size()-1;
        terms = new Complex[pow];
        Arrays.fill(terms, Complex.ORIGIN);
        for(Complex fract: systemSolver(equations)) add(fract, pow--);
    }
    // solves a system of linear equations where each equation equations[i] is
    // v1*equations[i][0]+v2*equations[i][1]... = equations[i][equations.size()-1]
    // returns [v1, v2 ...] but complex
    public static List<Complex> systemSolver(List<List<Complex>> equations){
        for(int i = 0; i<equations.size(); i++) if(equations.get(i).size() != equations.get(0).size())
            throw new IllegalArgumentException("Invalid equations");
        if(equations.size()+1 != equations.get(0).size())
            throw new IllegalArgumentException("Invalid equations");
        List<Complex> ans = new ArrayList<>(equations.get(0).size());
        int size = equations.size();
        for(int i = 0; i<size; i++) {
            List<List<Complex>> cloned = new ArrayList<>(equations.size());
            for(List<Complex> list: equations) cloned.add((List<Complex>)((ArrayList<Complex>)list).clone());
            Complex a = Complex.NaN;
            while(a.isNaN()) {
                if(cloned.size() == 1)
                    a = cloned.get(0).get(1).divide(cloned.get(0).get(0));
                else for(int j = 0; j<cloned.size()-1; j++) {
                    Complex save = cloned.get(j).get(cloned.get(j).size()-2);
                    for(int k = 0; k<cloned.get(j).size(); k++) {
                        Complex mul = cloned.get(j+1).get(cloned.get(j).size()-2);
                        Complex sub = save.multiply(cloned.get(j+1).get(k));
                        cloned.get(j).set(k, (cloned.get(j).get(k).multiply(mul).subtract(sub)));
                    }
                    cloned.get(j).remove(cloned.get(j).size()-2);
                }
                cloned.remove(cloned.size()-1);
            }
            ans.add(a);
            equations.remove(equations.size()-1);
            for(int j = 0; j<equations.size(); j++) {
                Complex sub = ans.get(ans.size()-1).multiply(equations.get(j).get(0));
                Complex put = equations.get(j).get(equations.get(j).size()-1).subtract(sub);
                equations.get(j).set(equations.get(j).size()-1, put);
                equations.get(j).remove(0);
            }
        }
        return ans;
    }

    public Complex[] getTerms(){
        return terms;
    }

    // returns the derivative of the polynomial in the real axis
    public Polynomial realDerivative(){
        Polynomial p = new Polynomial(degree()-1);
        for(int i = 1; i<=degree(); i++) p.terms[i-1] = new Complex(terms[i].real()*i, 0);
        return p;
    }
    // returns the derivative of the polynomial in the imaginary axis
    public Polynomial imagDerivative(){
        Polynomial p = new Polynomial(degree()-1);
        for(int i = 1; i<=degree(); i++) p.terms[i-1] = new Complex(0, terms[i].imag()*i);
        return p;
    }
    // returns the integral of the polynomial
    public Polynomial integral() {
        Polynomial p = new Polynomial(degree()+1);
        for(int i = 0; i<=degree(); i++) p.terms[i+1] = terms[i].divide(i+1);
        return p;
    }
    private Complex muller() {
        Complex x1 = Complex.ORIGIN, x2 = Complex.ONE, x3 = x2.multiply(2);
        for(boolean v = true; true; v = false) {
            Complex y1 = evaluate(x1), y2 = evaluate(x2), y3 = evaluate(x3);
            if(v) {
                if(y1.radius() <= 1e-5) return x1;
                if(y2.radius() <= 1e-5) return x2;
            }
            if(y3.radius() <= 1e-5) return x3;
            Complex q = x3.subtract(x2).divide(x2.subtract(x1));
            Complex a1 = q.multiply(y3);
            Complex a2 = q.multiply(q.add(1)).multiply(y2);
            Complex a3 = q.pow(2).multiply(y1);
            Complex a = a1.subtract(a2).add(a3);
            Complex b1 = q.multiply(2).add(1).multiply(y3);
            Complex b2 = q.add(1).pow(2).multiply(y2);
            Complex b3 = q.pow(2).multiply(y1);
            Complex b = b1.subtract(b2).add(b3);
            Complex c = q.add(1).multiply(y3);
            Complex sqrt = b.pow(2).subtract(a.multiply(c).multiply(4)).pow(0.5);
            Complex div = Complex.max(b.add(sqrt), b.subtract(sqrt));
            if(div.radius() <= 1e-5) {
                x3 = x3.multiply(1.1);
                continue;
            }
            Complex inside = c.multiply(2).divide(div);
            Complex next = x3.subtract(x3.subtract(x2).multiply(inside));
            x1 = x2;
            x2 = x3;
            x3 = next;
            if(x3.isNaN()) return x3;
        }
    }
    // finds all x values where evaluate(x) == 0
    public List<Complex> solve(){
        List<Complex> ret = new ArrayList<>(degree());
        Polynomial curr = clone();
        while(curr.degree() >= 1){
            Complex c = curr.muller();
            if(c.isNaN()) return ret;
            Polynomial v = new Polynomial(1);
            v.add(1, 1);
            v.add(c.negate(), 0);
            curr = curr.divide(v)[0];
            ret.add(c);
        }
        return ret;
    }
    // return degree of polynomial
    public int degree() {return terms.length-1;}
    // divides by another polynomial, returns [Quotient, Remainder]
    public Polynomial[] divide(Polynomial other){
        Polynomial[] ans = new Polynomial[2];
        ans[0] = new Polynomial();
        Polynomial div = clone();
        ans[1] = div;
        int max = other.degree();
        while(other.get(max).radius() <= 1e-8) max--;
        Complex get = other.get(max);
        for(int i = div.degree(); i>=max; i--) if(div.get(i).radius() > 1e-8){
            Complex g = div.get(i).divide(get);
            int pow = i-max;
            for(int j = 0; j<=max; j++)
                div.add(other.get(j).multiply(g).negate(), j+pow);
            ans[0].add(g, pow);
        }
        return ans;
    }
    private void resize(int size) {
        int p = terms.length;
        terms = Arrays.copyOf(terms, size);
        while(p < size) terms[p++] = Complex.ORIGIN;
    }
    // adds coeff*x^power
    public void add(double coeff, int power){
        if(power >= terms.length) resize(power+1);
        terms[power] = terms[power].add(coeff);
    }
    // adds coeff*x^power with complex numbers
    public void add(Complex coeff, int power) {
        if(power >= terms.length) resize(power+1);
        terms[power] = terms[power].add(coeff);
    }
    // add polynomial
    public void add(Polynomial poly){
        if(poly.degree() > degree()) resize(poly.degree()+1);
        for(int i = 0; i<=poly.degree(); i++) terms[i] = terms[i].add(poly.get(i));
    }
    // subtract polynomial
    public void subtract(Polynomial poly){
        if(poly.degree() > degree()) resize(poly.degree()+1);
        for(int i = 0; i<=poly.degree(); i++) terms[i] = terms[i].subtract(poly.get(i));
    }
    // multiplies by coeff
    public void multiply(double coeff){
        for(int i = 0; i<terms.length; i++) terms[i] = terms[i].multiply(coeff);
    }
    // multiplies by coeff complex
    public void multiply(Complex coeff){
        for(int i = 0; i<terms.length; i++) terms[i] = terms[i].multiply(coeff);
    }
    // multiplies by coeff*x^power
    public void multiply(double coeff, int power) {
        Complex[] t2 = new Complex[terms.length+power];
        Arrays.fill(t2, Complex.ORIGIN);
        for(int i = 0; i<=degree()-power; i++)
            t2[i+power] = terms[i].multiply(coeff);
        terms = t2;
    }
    // multiplies by coeff*x^power complex
    public Polynomial multiply(Complex coeff, int power) {
        Complex[] t2 = new Complex[terms.length+power];
        Arrays.fill(t2, Complex.ORIGIN);
        for(int i = 0; i<=degree()-power; i++)
            t2[i+power] = terms[i].multiply(coeff);
        return new Polynomial(terms);
    }
    // polynomial multiplcation in O(n^2)
    public Polynomial multiply(Polynomial poly) {
        Complex[] result = new Complex[poly.degree()+degree()+1];
        Arrays.fill(result, Complex.ORIGIN);
        for(int i = 0; i<=degree(); i++) for(int j = 0; j<=poly.degree(); j++)
            result[i+j] = result[i+j].add(terms[i].multiply(poly.terms[j]));
        return new Polynomial(result);
    }
    // returns the coefficient of the term with degree power
    public Complex get(int power){
        return power >= terms.length ? Complex.ORIGIN:terms[power];
    }
    // clones polynomial
    @Override
    public Polynomial clone(){
        Polynomial ans = new Polynomial(degree());
        for(int i = 0; i<=degree(); i++) ans.terms[i] = terms[i];
        return ans;
    }
    // converts polynomial to string, dosen't display term if the coefficient for that term is < precision
    public String toString(double precision) {
        StringBuilder s = new StringBuilder();
        boolean first = true;
        for(int i = degree(); i>=0; i--){
            Complex get = terms[i];
            if(get.radius() <= precision) continue;
            if(Math.abs(terms[i].imag()) <= precision) {
                double num = terms[i].real();
                boolean add = false;
                if(num != 1 || i == 0) {
                    if(s.length() != 0 && num > 0) s.append("+");
                    s.append(num);
                    add = true;
                }
                if(i == 1) s.append(first || add? "x":"+x");
                else if(i > 1) s.append("x^"+i);
            }else {
                if(s.length() != 0) s.append("+");
                s.append("("+get.toString()+")");
                if(i == 1) s.append("x");
                else if(i > 1) s.append("x^"+i);
            }
            first = false;
        }
        return s.toString();
    }
    // converts polynomial to string with precision 10^-6
    @Override
    public String toString(){return toString(1e-8);}
    // multiplies polynomials in O(nlogn) with slight inaccuracy
    public Polynomial FFTmultiply(Polynomial other){
        int l = 1; while(l <= Math.max(terms.length, other.terms.length)) l<<=1;
        Complex[] a = new Complex[l], b = new Complex[l];
        for(int i = 0; i<a.length; i++){
            a[i] = get(l-i-1);
            b[i] = other.get(l-i-1);
        }
        Complex[] get = convolve(a, b);
        Polynomial ret = new Polynomial(get.length);
        for(int i = 0; i<get.length-1; i++) ret.add(get[i].real(), get.length-i-2);
        return ret;
    }
    private static Complex[] fft(Complex[] x) {
        int n = x.length;
        if (n == 1) return new Complex[] { x[0] };
        Complex[] even = new Complex[n/2];
        for (int k = 0; k < n/2; k++)
            even[k] = x[2*k];
        Complex[] evenFFT = fft(even);
        Complex[] odd  = even;
        for (int k = 0; k < n/2; k++)
            odd[k] = x[2*k + 1];
        Complex[] oddFFT = fft(odd);
        Complex[] y = new Complex[n];
        for (int k = 0; k < n/2; k++) {
            double kth = -2 * k * Math.PI / n;
            Complex wk = new Complex(Math.cos(kth), Math.sin(kth));
            y[k] = evenFFT[k].add(wk.multiply(oddFFT[k]));
            y[k+n/2] = evenFFT[k].subtract(wk.multiply(oddFFT[k]));
        }
        return y;
    }
    private static Complex[] ifft(Complex[] x) {
        int n = x.length;
        Complex[] y = new Complex[n];
        for (int i = 0; i < n; i++)
            y[i] = x[i].conjugate();
        y = fft(y);
        for (int i = 0; i < n; i++)
            y[i] = y[i].conjugate();
        for (int i = 0; i < n; i++)
            y[i] = y[i].multiply(1.0 / n);
        return y;
    }
    private static Complex[] cconvolve(Complex[] x, Complex[] y) {
        int n = x.length;
        Complex[] a = fft(x);
        Complex[] b = fft(y);
        Complex[] c = new Complex[n];
        for (int i = 0; i < n; i++)
            c[i] = a[i].multiply(b[i]);
        return ifft(c);
    }
    private static Complex[] convolve(Complex[] x, Complex[] y) {
        Complex[] a = new Complex[x.length<<1];
        for (int i = 0; i<x.length; i++) a[i] = x[i];
        for (int i = x.length; i<x.length<<1; i++) a[i] = Complex.ORIGIN;
        Complex[] b = new Complex[2*y.length];
        for (int i = 0; i<y.length; i++) b[i] = y[i];
        for (int i = y.length; i<y.length<<1; i++) b[i] = Complex.ORIGIN;
        return cconvolve(a, b);
    }
    private static Polynomial sigma(int pow){
        Polynomial ans = new Polynomial(pow), expansion = new Polynomial(pow);
        if(pow == 0){
            ans.add(1, 1);
            return ans;
        }
        int prev = 1;
        for(int i = pow; i>0; i--){
            expansion.add(prev, i);
            prev = (prev*i)/(pow-i+1);
        }
        expansion.add(1, 0);
        for(int i = pow; i>=0; i--){
            Complex rem = Complex.ORIGIN;
            for(int j = 0; j<=ans.degree(); j++) if(ans.get(j).radius() > 1e-8)
                rem = rem.add(ans.get(j).multiply(nCk(j, i)));
            ans.add(expansion.get(i).subtract(rem).divide(i+1), i+1);
        }
        return ans;
    }
    public static double nCk(long n, long k){
        double ans = 1;
        for(int i = 0; i<k; i++) ans = (ans*(n-i))/(i+1);
        return ans;
    }
    // negates polynomial
    public void negate() {
        for(int i = 0; i<=degree(); i++) terms[i] = terms[i].negate();
    }
    // maps point a to origin by shifting all points
    public Polynomial shift(double d) {return shift(new Complex(d, 0));}
    // maps point a+bi to origin by shifting all points
    public Polynomial shift(Complex c) {
        c = c.negate();
        Polynomial ans = new Polynomial(degree());
        for(int i = 0; i<=degree(); i++) if(terms[i].radius() > 1e-8) {
            Polynomial curr = new Polynomial(new Complex[] {c, Complex.ONE}).pow(i);
            curr.multiply(terms[i]);
            ans.add(curr);
        }
        return ans;
    }
    // return a polynomial poly where poly.evaluate(x) is the sum of this.evaluate(n) where 0 <= n <= x
    // to use negative numbers and find the sum x <= n <= 0, you can do 
    // poly = poly.shift(10).realDiscreteIntegral();
    // System.out.println(poly.evaluate(10).subtract(poly.evaluate(4)))
    // that will get the sum -5 <= n <= 0
    public Polynomial discreteIntegral(){
        Polynomial change = new Polynomial(degree()<<1);
        for(int i = 0; i<=degree(); i++) if(terms[i].radius() > 1e-8){
            Polynomial curr = sigma(i);
            switch(i&3) {
                case 1:
                    curr.multiply(Complex.I);
                    break;
                case 2:
                    curr.multiply(Complex.NEGONE);
                    break;
                case 3:
                    curr.multiply(Complex.NEGI);
            }
            curr.multiply(terms[i]);
            change.add(curr);
        }
        change.add(terms[0], 0);
        change = change.compose(new Polynomial(new Complex[] {Complex.ORIGIN, Complex.NEGI}));
        return change;
    }
    public Polynomial discreteIntegralHelper() {
        Polynomial change = new Polynomial(degree()<<1);
        for(int i = 0; i<=degree(); i++) if(terms[i].radius() > 1e-8){
            Polynomial curr = sigma(i);
            curr.multiply(terms[i]);
            change.add(curr);
        }
        change.add(terms[0], 0);
        return change;
    }
    // raises polynomial to an exponent
    public Polynomial pow(int pow) {
        if(pow < 0) throw new IllegalArgumentException("Invalid power");
        Polynomial ret = new Polynomial(new Complex[] {Complex.ONE}), clone = clone();
        while(pow > 0) {
            if((pow&1) == 1) ret = ret.multiply(clone);
            clone = clone.multiply(clone);
            pow>>=1;
        }
        return ret;
    }
    // raises polynomial to an exponent using fft
    public Polynomial FFTpow(int pow) {
        if(pow < 0) throw new IllegalArgumentException("Invalid power");
        Polynomial ret = new Polynomial(new Complex[] {Complex.ONE}), clone = clone();
        while(pow > 0) {
            if((pow&1) == 1) ret = ret.multiply(clone);
            clone = clone.FFTmultiply(clone);
            pow>>=1;
        }
        return ret;
    }
    // returns a new polynomial being this(other(x))
    public Polynomial compose(Polynomial other) {
        Polynomial poly = new Polynomial();
        boolean started = false;
        for (int i = degree(); i>=0; i--) if(terms[i].radius() > 1e-8 || started){
            poly = poly.multiply(other);
            poly.add(terms[i], 0);
            started = true;
        }
        return poly;
    }
    // returns a new polynomial being this(other(x)) using fft
    public Polynomial FFTcompose(Polynomial other) {
        Polynomial poly = new Polynomial();
        boolean started = false;
        for (int i = degree(); i>=0; i--) if(terms[i].radius() > 1e-8 || started){
            poly = poly.FFTmultiply(other);
            poly.add(terms[i], 0);
            started = true;
        }
        return poly;
    }
    // plugs c into the polynomial
    public Complex evaluate(double c) {
        Complex ans = Complex.ORIGIN;
        for(int i = 0; i<=degree(); i++)
            ans = ans.add(terms[i].multiply(Math.pow(c, i)));
        return ans;
    }    // plugs c into the polynomial complex
    public Complex evaluate(Complex c) {
        Complex ans = Complex.ORIGIN;
        for(int i = 0; i<=degree(); i++)
            ans = ans.add(c.pow(i).multiply(terms[i]));
        return ans;
    }
    // returns all local maximums and minimums of the function
    public Complex[] maxMin(){
        List<Complex> real = realDerivative().solve();
        List<Complex> imag = imagDerivative().solve();
        Complex[] ret = new Complex[real.size()*imag.size()];
        int p = 0;
        for(int i = 0; i<real.size(); i++) for(int j = 0; j<imag.size(); j++)
            ret[p++] = real.get(i).add(imag.get(j));
        return ret;
    }
    // checks if polynomials are equal with coefficient precision of precision
    public boolean equals(Object other, double precision) {
        if(other == this) return true;
        if(!(other instanceof Polynomial)) return false;
        Complex[] t = ((Polynomial)other).terms;
        if(t.length > terms.length)
            for(int i = terms.length; i<t.length; i++) if(t[i].radius() > precision) return false;
        if(t.length < terms.length)
            for(int i = t.length; i<terms.length; i++) if(terms[i].radius() > precision) return false;
        for(int i = 0; i<Math.min(terms.length, t.length); i++) if(terms[i].subtract(t[i]).radius() > precision)
            return false;
        return true;
    }
    // checks if polynomials are equal with precision 1e-8
    @Override
    public boolean equals(Object other) {return equals(other, 1e-8);}
    // compares polynomials by max degree, then by each coefficient
    @Override
    public int compareTo(Polynomial o) {
        if(o.degree() != degree()) return Integer.compare(degree(), o.degree());
        for(int i = degree(); i>=0; i--) {
            int c = get(i).compareTo(o.get(i));
            if(c == 0) return c;
        }
        return 0;
    }
}