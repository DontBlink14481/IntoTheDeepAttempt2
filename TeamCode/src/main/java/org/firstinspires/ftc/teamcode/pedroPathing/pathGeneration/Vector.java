package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

/**
 * This is the Point class. This class handles storing information about vectors, which are
 * basically Points but using polar coordinates as the default. The main reason this class exists
 * is because some vector math needs to be done in the Follower, and dot products and cross
 * products of Points just don't seem right. Also, there are a few more methods in here that make
 * using Vectors a little easier than using a Point in polar coordinates.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/11/2024
 */
public class Vector {

    // IMPORTANT NOTE: theta is defined in radians.
    // These are the values of the coordinate defined by this Point, in both polar and
    // Cartesian systems.
    private double magnitude;
    private double theta;
    private double xComponent;
    private double yComponent;

    /**
     * This creates a new Vector with zero magnitude and direction.
     */
    public Vector() {
        setComponents(0, 0);
    }

    /**
     * This creates a new Vector with a specified magnitude and direction.
     *
     * @param magnitude magnitude of the Vector.
     * @param theta the direction of the Vector in radians.
     */
    public Vector(double magnitude, double theta) {
        setComponents(magnitude, theta);
    }

    public Vector(double a, double b, boolean polar){
        if(polar) setComponents(a, b);
        else setOrthogonalComponents(a, b);
    }

    /**
     * This sets the components of the Vector in regular vector coordinates.
     *
     * @param magnitude sets the magnitude of this Vector.
     * @param theta sets the theta value of this Vector.
     */
    public void setComponents(double magnitude, double theta) {
        double[] orthogonalComponents;
        if (magnitude<0) {
            this.magnitude = -magnitude;
            this.theta = MathFunctions.normalizeAngle(theta+Math.PI);
        } else {
            this.magnitude = magnitude;
            this.theta = MathFunctions.normalizeAngle(theta);
        }
        orthogonalComponents = Point.polarToCartesian(magnitude, theta);
        xComponent = orthogonalComponents[0];
        yComponent = orthogonalComponents[1];
    }

    /**
     * This sets only the magnitude of the Vector.
     *
     * @param magnitude sets the magnitude of this Vector.
     */
    public void setMagnitude(double magnitude) {
        setComponents(magnitude, theta);
    }

    /**
     * This sets only the angle, theta, of the Vector.
     *
     * @param theta sets the angle, or theta value, of this Vector.
     */
    public void setTheta(double theta) {
        setComponents(magnitude, theta);
    }

    /**
     * This rotates the Vector by an angle, theta.
     *
     * @param theta2 the angle to be added.
     */
    public void rotateVector(double theta2) {
        setTheta(theta+theta2);
    }

    /**
     * This sets the orthogonal components of the Vector. These orthogonal components are assumed
     * to be in the direction of the x-axis and y-axis. In other words, this is setting the
     * coordinates of the Vector using x and y coordinates instead of a direction and magnitude.
     *
     * @param xComponent sets the x component of this Vector.
     * @param yComponent sets the y component of this Vector.
     */
    public void setOrthogonalComponents(double xComponent, double yComponent) {
        double[] polarComponents;
        this.xComponent = xComponent;
        this.yComponent = yComponent;
        polarComponents = Point.cartesianToPolar(xComponent, yComponent);
        magnitude = polarComponents[0];
        theta = polarComponents[1];
    }

    /**
     * This sets the x value.
     *
     * @param set the x value
     */
    public void setXComponent(double set) {
        xComponent = set;
    }

    /**
     * This sets the y value.
     *
     * @param set the y value
     */
    public void setYComponent(double set) {
        yComponent = set;
    }

    /**
     * Returns the magnitude of this Vector.
     *
     * @return returns the magnitude.
     */
    public double getMagnitude() {
        return magnitude;
    }

    /**
     * Returns the theta value, or angle, of this Vector.
     *
     * @return returns the theta value.
     */
    public double getTheta() {
        return theta;
    }

    /**
     * Returns the x component of this Vector.
     *
     * @return returns the x component.
     */
    public double getXComponent() {
        return xComponent;
    }

    /**
     * Returns the y component of this Vector.
     *
     * @return returns the y component.
     */
    public double getYComponent() {
        return yComponent;
    }

    /**
     * This creates a copy of this Vector that points to a new memory location.
     *
     * @return returns a deep copy of this Vector
     */
    public Vector copy() {
        return new Vector();
    }

    public Vector times(double d){
        return new Vector(d * getXComponent(), d * getYComponent());
    }
    public Vector plus(Vector v){
        return new Vector(getXComponent() + v.getXComponent(), getYComponent() + v.getYComponent());
    }
    public Vector minus(Vector v){
        return plus(new Vector(-v.xComponent, -v.yComponent));
    }
    public Vector rotated(double rad){
        return new Vector(getMagnitude(), getTheta() + rad);
    }
}