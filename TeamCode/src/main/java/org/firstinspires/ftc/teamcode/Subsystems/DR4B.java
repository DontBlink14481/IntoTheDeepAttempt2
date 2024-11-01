package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Double.NaN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.MotionProfile;
import org.firstinspires.ftc.teamcode.Util.Util;

@Config

public class DR4B implements Subsystem {

    //constants
    private static final double TICKS_PER_REV = 8192;
    private static final double GEAR_RATIO = 1.0;
    public static final double HIGHEST_ANGLE = 1.72;
    public static final double HIGHEST_WE_SHOULD_GO = HIGHEST_ANGLE;

    //power stuff
    public static double LOWER_POWER_BOUND = -0.6;
    public static double UPPER_POWER_BOUND = 0.8;
    public static double NEUTRAL_POWER = -0.1;

    //mp
    public static double maxAccel = 3.07920107632;
    public static double maxVel = 6.062321569464546e3;
    public static double maxDropAccel = 7.07920107632;
    public static double kv = 3.5e-1;
    public static double ka = 6e-2;
    public static double accSatErr = 0;
    public static double mpResetError = -1;
    public static boolean motionProfile = true;

    //gains
    public static double kp = 2;
    public static double ki = 0;
    public static double kff = 0.02;
    public static double kg = 0.00;
    public static double kfa = 0.2;
    public static double kd = 0.1;
    public static double iResetThreshold = 0.01;
    public static boolean smartDamp = false;
    public static boolean smarterDamp = false;

    //hardware
    private final HardwareMap hardwareMap;
    public DcMotorEx left;
    public DcMotorEx right;
    public DcMotor encoder;

    //consistent variables
    public int iResetAmount = 0;
    private double prevVel = 0;
    private double startTime = 0;
    private double startPos = 0;
    public MotionProfile currentMp = null;
    public double velocity = 0;
    public double acceleration = 0;
    private double prev_error = 0;
    private double prev_time = 0;
    public double prevPos = 0;
    public boolean useDropAccel = false;
    public double totalI = 0;
    public double position = 0;

    public static double UPPER_SPECIMEN = 0;
    public static double LOWER_SPECIMEN = 0;
    public static double UPPER_BASKET = 0;
    public static double LOWER_BASKET = 0;

    public DR4B(HardwareMap h) {
        hardwareMap = h;
        left = hardwareMap.get(DcMotorEx.class, "ldr4b");
        right = hardwareMap.get(DcMotorEx.class, "rdr4b");
        encoder = hardwareMap.get(DcMotor.class, "ldr4b"); // encoder in left front motor spot

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setDirection(DcMotorSimple.Direction.FORWARD);

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void reset() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void toInit() {
    }

    public static double cosTheta(double angle) {
        if (angle > HIGHEST_WE_SHOULD_GO) return 0;
        return Math.cos(angle * 2 * Math.PI / (4 * HIGHEST_WE_SHOULD_GO));
    }

    public double pid(boolean useVoltage){
        if(!useVoltage) return pid();
        return pid();
    }

    public double pid() {
        double curr_time = System.nanoTime()/1E9;
        // Comment the below value when using the V4B Tester (It should be good to go for V4B tuner
        double curr_error = /*getRowPosition((int) currRow)*/position - getAngle();
        double kd = DR4B.kd;
        if(smartDamp) kd = SMART_DAMP(kp, kv, ka);
        else if(smarterDamp) kd = SMARTER_DAMP(kp, kv, ka);
        velocity = (position - prevPos) / (curr_time - prev_time);
        acceleration = (velocity - prevVel) / (curr_time - prev_time);

        if(Util.isCloseEnough(velocity, 0, mpResetError)){
            startPos = getAngle();
            startTime = curr_time;
        }

        double mp = 0;
        if (motionProfile) {
            currentMp = MotionProfile.trapMotion(curr_time - startTime, (useDropAccel) ? maxDropAccel : maxAccel,maxVel, startPos, position);
            curr_error = currentMp.goalPosition - getAngle();
            mp += ka * currentMp.goalAcceleration + kv * currentMp.goalVelocity;
        }



        double p = DR4B.kp * curr_error;
        double d = kd * (curr_error - prev_error) / (curr_time - prev_time);
        double FF = kff * (Math.signum(curr_error));
        double FA = kfa * cosTheta(getAngle());
        if(!Util.isCloseEnough(acceleration, 0, accSatErr))totalI += ki * (curr_error + prev_error) * (curr_time - prev_time) / 2;
        if (Math.abs(curr_error) < iResetThreshold) {
            totalI = 0;
            iResetAmount++;
        }

        double power = p + d + FF + FA + mp + totalI;
        if (getAngle() - prevPos < 0) {
            power += kg;
        }

        prev_error = curr_error;
        prev_time = curr_time;
        prevPos = getAngle();
        // Comment the below value when using the V4B Tester

        return power;
    }

    @Override
    public void update() {
        //TODO: Polarity of motors is wrong or something -> setRawPowijher()
        if(getAngle() < 0){
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //if its hanging negative power is applied
        //if not, neutral down power or pid power is applied
        setRawPower((position == 0.0) ? NEUTRAL_POWER : pid());
    }

    // set position for testing based on position
    public void setPosition(double pos) {
        if (pos != position) {
            useDropAccel = pos <= getAngle();
            position = pos;
            totalI = 0;
            iResetAmount++;
            startTime = System.nanoTime()/1E9;
            startPos = getAngle();
//            useSUSPID = position - getAngle() < 0.2;
        }
    }

    public void update(Telemetry t) {
        update();
//        t.addData("using suspid", useSUSPID);
        t.addData("v4b internal position", position);
        t.addData("equal to 0", position == 0.0);
    }

    public void setRawPower(double pwm) {
        pwm = Range.clip(pwm, LOWER_POWER_BOUND, UPPER_POWER_BOUND);
        setUnrestrictedRawPower(pwm);
    }

    public static double SMART_DAMP(double Kp, double Kv, double Ka){
        //smartdamp for kp >= kv^2/(4ka)
        return Math.max(2 * Math.sqrt(Ka * Kp) - Kv, 0);
    }

    public static double SMARTER_FRICTION(double Kp, double Ka){
        return Math.pow(Math.sqrt(Kp/(3*Ka)), 3);
    }

    public static double SMARTER_DAMP(double Kp, double Kv, double Ka){
        double pole = -Math.sqrt(Kp/(3*Ka));
        return Math.max(0, -pole*3*Ka - Kv);
    }

    public void setUnrestrictedRawPower(double pwm) {
        left.setPower(pwm);
        right.setPower(-pwm);
    }

    public static double ticksToRad(double ticks) {
        return 2.0 * Math.PI * (ticks / (TICKS_PER_REV * GEAR_RATIO));
    }

    public double getPosition() {
        return position;
    }


    public double getAngle() {
        return ticksToRad(encoder.getCurrentPosition());
    }


}
