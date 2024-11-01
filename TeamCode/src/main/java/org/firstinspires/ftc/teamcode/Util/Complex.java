package org.firstinspires.ftc.teamcode.Util;

public class Complex extends Number implements Comparable<Complex>{
	private final double real, imag;
	public static final Complex ORIGIN = new Complex(0, 0);
	public static final Complex ONE = new Complex(1, 0);
	public static final Complex NEGONE = new Complex(-1, 0);
	public static final Complex I = new Complex(0, 1);
	public static final Complex NEGI = new Complex(0, -1);
	public static final Complex NaN = new Complex(Double.NaN, Double.NaN);
	public static final Complex E = new Complex(Math.E, 0);
	// creates complex number with re^itheta
	public static Complex rect(double radius, double angle) {return new Complex(angle).multiply(radius);}
	// creates complex number with real part
	public static Complex real(double real) {return new Complex(real, 0);}
	// creates complex number with imaginary part
	public static Complex imag(double imag) {return new Complex(0, imag);}
	// creates complex number with real and imaginary part
	public Complex(double real, double imag) {
		this.real = real;
		this.imag = imag;
	}
	public boolean isNaN() {return real != real || imag != imag;}
	// finds complex number on the complex unit circle with a theta value of angle
	public Complex(double angle) {this(Math.cos(angle), Math.sin(angle));}
	// finds cube root
	public Complex cbrt() {return pow(1D/3D);}
	// finds square root
	public Complex sqrt() {return pow(0.5);}
	// converts string to complex number
	public Complex(String s) {
		if(!s.contains("i")) {
			real = Double.valueOf(s);
			imag = 0;
			return;
		}
		if(s.length() == 1) {
			real = 0;
			imag = 1;
			return;
		}
		int swap = -1;
		for(int i = 1; i<s.length(); i++) if(s.charAt(i) == '-' || s.charAt(i) == '+') {
			swap = i;
			break;
		}
		if(swap == -1) {
			imag = Double.valueOf(s.substring(0, s.length()-1));
			real = 0;
			return;
		}
		real = Double.valueOf(s.substring(0, swap));
		imag = swap == s.length()-1 ? 0:Double.valueOf(s.substring(swap, s.length()-1));
	}
	// adds complex numbers
	public Complex add(Complex c) {return new Complex(real+c.real, imag+c.imag);}
	// adds value to complex number
	public Complex add(double i) {return new Complex(real+i, imag);}
	// subtracts complex numbers
	public Complex subtract(Complex c) {return new Complex(real-c.real, imag-c.imag);}
	// subtracts value from complex number
	public Complex subtract(double i) {return new Complex(real-i, imag);}
	// multiplies complex numbers
	public Complex multiply(Complex c) {return new Complex(real*c.real-imag*c.imag, real*c.imag+imag*c.real);}
	// multiplies complex number by value
	public Complex multiply(double i) {return new Complex(real*i, imag*i);}
	// divides complex numbers
    public Complex divide(Complex c) {    	
        double d = c.real*c.real+c.imag*c.imag;
        double re = (real*c.real+imag*c.imag)/d;
        double im = (imag*c.real-real*c.imag)/d;
        return new Complex(re, im);
    }
    // divides complex number by value
	public Complex divide(double i) {return new Complex(real/i, imag/i);}
	// raises complex number to real exponent
	public Complex pow(double i) {return new Complex(angle()*i).multiply(Math.pow(radius(), i));}
	// raises complex number to complex exponent
	public Complex pow(Complex c) {
		Complex x = pow(c.real);
		double a = c.imag*Math.log(radius());
		Complex y = new Complex(a).multiply(Math.exp(-c.imag*angle()));
		return x.multiply(y);
	}
	// multiplies complex number by -1
	public Complex negate() {return new Complex(-real, -imag);}
	// does 1/complex number
	public Complex reciprocal() {return new Complex(1, 0).divide(this);}
	// finds distance from complex number to origin
	public double radius() {return Math.hypot(real, imag);}
	// finds angle of complex number
	public double angle() {return Math.atan2(imag, real);}
	// rotates angle of complex number while maintaining radius
	public Complex rotate(double angle) {return new Complex(angle+angle()).multiply(radius());}
	// computes natural log of complex number
	public Complex ln(){return new Complex(Math.log(radius()), angle());}
	// computes log of complex number with base
	public Complex log(Complex base){return ln().divide(base.ln());}
	// finds (real^2-imaginary^2)/complex number
	public Complex conjugate(){return new Complex(real, -imag);}
	// finds real part of number
	public double real(){return real;}
	// finds imaginary part of number
	public double imag(){return imag;}
	public static Complex max(Complex... arr) {
		Complex ans = arr[0];
		for(int i = 1; i<arr.length; i++) if(arr[i].compareTo(ans) == 1) ans = arr[i];
		return ans;
	}
	public static Complex min(Complex... arr) {
		Complex ans = arr[0];
		for(int i = 1; i<arr.length; i++) if(arr[i].compareTo(ans) == -1) ans = arr[i];
		return ans;
	}
	public Complex square() {return multiply(this);}
	public Complex cube() {return multiply(square());}
	// checks if numbers are equal
	@Override
	public boolean equals(Object obj) {
		if(!(obj instanceof Complex)) return false;
		Complex c = (Complex)obj;
		if(c.isNaN() && isNaN()) return true;
		return c.real == real && c.imag == imag;
	}
	// converts number to string
	@Override
	public String toString() {return toString(1e-6);}
	// converts number to string with precision
	public String toString(double precision) {
		if(Math.abs(imag) <= precision) return ""+real;
		if(Math.abs(real) <= precision) return imag+"i";
		return real+(imag < 0 ? "":"+")+imag+"i";
	}
	// converts real part to integer
	@Override
	public int intValue() {return (int)real;}
	// converts real part to long
	@Override
	public long longValue() {return (long)real;}
	// converts real part to float
	@Override
	public float floatValue() {return (float)real;}
	// converts real part to double
	@Override
	public double doubleValue() {return real;}
	// compares complex numbers by their distance from origin
	@Override
	public int compareTo(Complex o) {return Double.compare(radius(), o.radius());}
	// takes complex sin
	public Complex sin() {return E.pow(multiply(I)).subtract(E.pow(multiply(NEGI))).divide(I.multiply(2));}
	// finds elementary complex inverse sine
	public Complex arcsin() {return NEGI.multiply(multiply(I).add(ONE.subtract(square()).sqrt()).ln());}
	// takes complex cosecant
	public Complex csc() {return sin().reciprocal();}
	// finds elementary inverse complex cosecant
	public Complex arcscs() {return reciprocal().arcsin();}
	// takes complex cosine
	public Complex cos() {return E.pow(multiply(I)).add(E.pow(multiply(NEGI))).divide(2);}
	// finds elementary complex inverse cosine
	public Complex arccos() {return NEGI.multiply(add(square().subtract(ONE).sqrt()).ln());}
	// takes complex secant
	public Complex sec() {return cos().reciprocal();}
	// finds elementary complex inverse secant
	public Complex arcsec() {return reciprocal().arccos();}
	// takes complex tangent
	public Complex tan() {return sin().divide(cos());}
	// finds elementary complex inverse tangent
	public Complex arctan() {return NEGI.divide(2).multiply(negate().add(I).divide(add(I)).ln());}
	// takes complex cotangent
	public Complex cot() {return tan().reciprocal();}
	// finds elementary inverse complex cotangent
	public Complex arccot() {return reciprocal().arctan();}
	// takes complex hyperbolic sin
	public Complex sinh() {return multiply(I).sin().multiply(NEGI);}
	// finds elementary complex inverse hyperbolic sin
	public Complex arcsinh() {return add(square().add(ONE).sqrt()).ln();}
	// takes complex hyperbolic cosecant
	public Complex csch() {return sinh().reciprocal();}
	// finds elementary complex inverse hyperbolic cosecant
	public Complex arccsch() {return reciprocal().arcsinh();}
	// takes complex hyperbolic cosine
	public Complex cosh() {return multiply(I).cos();}
	// finds elementary complex inverse hyperbolic cosine
	public Complex arccosh() {return add(square().subtract(ONE).sqrt()).ln();}
	// takes complex hyperbolic secant
	public Complex sech() {return cosh().reciprocal();}
	// finds elementary complex inverse hyperbolic secant
	public Complex arcsech() {return reciprocal().arccosh();}
	// takes complex hyperbolic tangent
	public Complex tanh() {return multiply(I).tan().multiply(NEGI);}
	// finds elementary complex inverse hyperbolic tangent
	public Complex arctanh() {return add(ONE).divide(negate().add(ONE)).ln().divide(2);}
	// takes complex hyperbolic cotangent
	public Complex coth() {return tanh().reciprocal();}
	// finds elementary complex inverse hyperbolic cotangent
	public Complex arccoth() {return reciprocal().arctanh();}
}