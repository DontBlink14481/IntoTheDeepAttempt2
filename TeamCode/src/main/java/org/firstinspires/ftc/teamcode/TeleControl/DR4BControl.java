package org.firstinspires.ftc.teamcode.TeleControl;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DR4B;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.FallingEdge;

public class DR4BControl implements Control {
    DR4B dr4b;
    Robot robot;

    Gamepad gp1;
    Gamepad gp2;

    public boolean outtakeOnRung = false;

    FallingEdge us = new FallingEdge(() -> {dr4b.setPosition(DR4B.UPPER_SPECIMEN); outtakeOnRung = true;});
    FallingEdge ls = new FallingEdge(() -> {dr4b.setPosition(DR4B.LOWER_SPECIMEN); outtakeOnRung = true;});
    FallingEdge ub = new FallingEdge(() -> {dr4b.setPosition(DR4B.UPPER_BASKET); outtakeOnRung = false;});
    FallingEdge lb = new FallingEdge(() -> {dr4b.setPosition(DR4B.LOWER_BASKET); outtakeOnRung = false;});

    public DR4BControl(DR4B d, Gamepad gp1, Gamepad gp2) {
        dr4b = d;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public DR4BControl(Robot r, Gamepad gp1, Gamepad gp2) {
        this(r.dr4b, gp1, gp2);
        robot = r;
    }

    @Override
    public void update() {
        us.updateOnPress(gp2.dpad_down);
        ls.updateOnPress(gp2.dpad_right);
        ub.updateOnPress(gp2.dpad_up);
        lb.updateOnPress(gp2.dpad_left);
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {

    }
}
