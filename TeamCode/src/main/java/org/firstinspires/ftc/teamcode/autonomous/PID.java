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
    private int targetCount = 10;

    private double limit = 1;

    public PID (double p, double i, double d) {
        P = p;
        I = i;
        D = d;
    }

    public void setLimit(double newLimit) {
        limit = newLimit;
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
        double error = setpoint - input;
        double integral = prevIntegral + error * (deltaT * 0.001);
        double derivative = (error - prevError) / (deltaT * 0.001);

        double output = P * error + I * integral + D * derivative + bias;
/*        while (output > limit) {
            error = (limit / error) * error;
            integral = prevIntegral + error * (deltaT * 0.001);
            derivative = (error - prevError) / (deltaT * 0.001);
            output = P * error + I * integral + D * derivative + bias;
        }

        output = P * error + I * integral + D * derivative + bias;*/

        prevError = error;
        if (output > limit) {

            prevIntegral = 0;
            return limit;
        } else  if (output < -limit){
            prevIntegral = 0;
            return -limit;
        } else {
            prevIntegral = integral;
            return output;
        }
    }

    public boolean atTarget(double input) {
        if (Math.abs((setpoint - input)) < tolerance) {
            onTargetCount += 1;
        } else {
            onTargetCount = 0;
        }

        return onTargetCount >= targetCount;
    }

    public void setOutputLimit(double value) {
        limit = value;
    }
}

