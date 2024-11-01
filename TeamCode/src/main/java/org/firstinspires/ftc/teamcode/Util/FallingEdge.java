package org.firstinspires.ftc.teamcode.Util;

public class FallingEdge {
    public boolean isPressed = false;
    public FallingFunc func;
    public boolean isRunning = false;
    public FallingEdge(FallingFunc f) {
        func = f;
    }

    public void update(boolean cond) {
        if (cond) {
            isPressed = true;
        }
        if (!cond && isPressed) {
            func.run();
            isRunning = true;
            isPressed = false;
        } else {
            isRunning = false;
        }
    }

    public void reset() {
        isPressed = false;
        isRunning = false;
    }

    @FunctionalInterface
    public interface FallingFunc {
        void run();
    }
}
