package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

@Config
public class IntakeSlides implements Subsystem {
    public DcMotorEx slideMotorR; // TODO: figure out encoder motor
    public DcMotorEx slideMotorL;
    public DcMotor slidesEncoder;
    VoltageSensor voltageSensor;
    private static final double TICKS_PER_REV = 145.1;//TODO: Update
    private static final double GEAR_RATIO = 1.0;
    public static double kp = 0.0015;
    public static double kd = 0.0000;
    public static double ki = 0;
    public double totalI = 0;
    public static double kf = 0.0;
    private double prev_error;
    private double prev_time;
    public static double admissible = 40;

    public static double NEUTRAL_POWER = -0.05;

    public double power = 0;

    public double position = 0;
    public static boolean rawPower = false;

    public static double IN = -200;
    public static double PARTIAL = 600;
    public static double EXTENDED = 1730;

    public static double optimal_power = 14;

    public IntakeSlides(HardwareMap map) {
        this(map, true);
    }

    public IntakeSlides(HardwareMap map, boolean resetEncoder){
        slideMotorR = map.get(DcMotorEx.class, "re");
        slideMotorL = map.get(DcMotorEx.class, "le");
        slidesEncoder = map.get(DcMotor.class, "le");
        voltageSensor = map.voltageSensor.iterator().next();
//        slidesEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorL.setDirection(DcMotorSimple.Direction.FORWARD);

        if(resetEncoder){
            slidesEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void reset() {
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static double ticksToRad(double ticks) {
        return 2.0 * Math.PI * (ticks / (TICKS_PER_REV * GEAR_RATIO));
    }


    public double pid() { // slides pid
        double curr_time = System.nanoTime() / 1E9; // nano -> sec
        double curr_error = position - getRealPosition();

        double pp = kp * curr_error;
        double pd = kd * (curr_error - prev_error) / (curr_time - prev_time);
        double pf = kf * Math.signum(curr_error);
        totalI += ki * curr_error * (curr_time - prev_time);

        power = (pp + pd + pf + totalI);

        prev_error = curr_error;
        prev_time = curr_time;
        if (Math.abs(curr_error) < admissible) power = 0;

        return power * (optimal_power / getVoltage());
    }

    public void setRawPower(double power) {
        rawPower = true;
        slideMotorR.setPower(power);
        slideMotorL.setPower(power);
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    public void toInit() {
        setPosition(IN);
    }

    @Override
    public void update() {
        if(rawPower) setRawPower(0);
        else runPID();
    }

    public void runPID() {
        slideMotorR.setPower(pid());
        slideMotorL.setPower(pid());
    }

    public void setPosition(double position) {
        rawPower = false;
        this.position = Range.clip(position, IN, EXTENDED);
    }

    public void slidesNeutral() {
        setRawPower(NEUTRAL_POWER);
    }

    public int getRealPosition() {
        return slidesEncoder.getCurrentPosition();
    }
    public double getAngle() {
        return ticksToRad(slidesEncoder.getCurrentPosition());
    }

}
