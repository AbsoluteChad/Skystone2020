package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PIDController {

    private final double LOOP_TIME = 0.02;

    private double P;
    private double I;
    private double D;

    private double kP;
    private double kI;
    private double kD;

    private double setPoint;
    private double error;
    private double prevError;

    public PIDController(double kP, double kI, double kD, double setPoint) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        P = 0;
        I = 0;
        D = 0;

        this.setPoint = setPoint;
        error = setPoint;
        prevError = 0;
    }

    public PIDController(PIDCoefficients PIDcoeffs, double setPoint) {
        this.kP = PIDcoeffs.p;
        this.kI = PIDcoeffs.i;
        this.kD = PIDcoeffs.d;

        P = 0;
        I = 0;
        D = 0;

        this.setPoint = setPoint;
        error = setPoint;
        prevError = 0;
    }

    public double motorOutput(double actual) {
        error = setPoint - actual;
        P = error * kP;
        I += (error * LOOP_TIME) * kI;
        D = ((error - prevError) / LOOP_TIME) * kD;
        prevError = error;
        return P + I + D;
    }

    public double getError() {
        return error;
    }
}
