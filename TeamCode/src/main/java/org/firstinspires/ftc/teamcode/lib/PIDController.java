package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private DcMotor[] motors;
    private ElapsedTime timer = new ElapsedTime();
    private double lastTimestamp = 0;

    private double kP;
    private double kI;
    private double kD;
    private double iCap;

    private double error = 100;
    private double errorSum = 0;
    private double prevError = 0;

    public PIDController(DcMotor[] motors, PIDCoefficients PIDcoeffs, double iCap) {
        this.motors = motors;

        this.kP = PIDcoeffs.p;
        this.kI = PIDcoeffs.i;
        this.kD = PIDcoeffs.d;
        this.iCap = iCap;

        timer.reset();
    }

    public void drive(double setPoint, double tolerance, boolean[] reverse) {
        while (Math.abs(error) > tolerance) {
            //Determine error
            double actual = motors[0].getCurrentPosition();
            error = setPoint - actual;

            //Determine dt (time delta) for time step compensation & apply to raw (w/o gains) I & D terms
            double dt = timer.milliseconds() - lastTimestamp;
            if (Math.abs(error) < iCap) {
                errorSum += error * dt;
            }
            double derivative = (error - prevError) / dt;

            //Determine motor output based off P, I, and D values
            double P = error * kP;
            double I = errorSum * kI;
            double D = derivative * kD;
            double output = P + I + D;

            //Apply powers to all motors
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(reverse[i] ? -output : output);
            }

            //Set conditions for next iteration
            prevError = error;
            lastTimestamp = timer.milliseconds();
        }

        //Stop all motors
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }
}
