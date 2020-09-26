package org.firstinspires.ftc.teamcode.autonomous;

public class PID {
    private double P;
    private double I;
    private double D;

    private double tolerance = 0.01;

    private double setpoint = 0;

    private double bias = 0;

    private double deltaT;

    private double prevError = 0;
    private double prevIntegral = 0;

    private int onTargetCount = 0;
    private int targetCount = 5;

    public PID (double p, double i, double d) {
        P = p;
        I = i;
        D = d;
    }

    public void setTargetCount(int count) {
        targetCount = count;
    }

    public void setDeltaT(double dt) {
        deltaT = dt;
    }

    public double getDeltaT() {
        return deltaT;
    }

    public void setTolerance(double tol) {
        tolerance = tol;
    }

    public void setPoint(double point) {
        setpoint = point;
    }

    public double doPID(double input) {
        double error = input - setpoint;
        double integral = prevIntegral + error * deltaT;
        double derivative = (error - prevError) / deltaT;

        double output = P * error + I * integral + D * derivative + bias;

        prevError = error;
        prevIntegral = integral;
        return output;
    }

    public boolean atTarget(double input) {
        if (Math.abs((setpoint - input)/setpoint) < tolerance) {
            onTargetCount += 1;
        } else {
            onTargetCount = 0;
        }

        return onTargetCount >= targetCount;
    }
}

