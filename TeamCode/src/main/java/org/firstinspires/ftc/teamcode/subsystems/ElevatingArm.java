package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.lib.drive.PIDController;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ElevatingArm extends Subsystem {

    //Declare private instance
    private static final ElevatingArm instance = new ElevatingArm();

    //Declare motors
    public DcMotor elevatorArmLeft;
    public DcMotor elevatorArmRight;
    public DcMotor rotationalArm;

    //Declare sensors
    //public TouchSensor outSensor;
    public DigitalChannel inSensor;

    //Declare constants
    private static final double VIDIPT_ELEVATOR_CONTROL = 0.75;
    private static final double VIDIPT_ROTATIONAL_ARM_CONTROL = 0.6;
    private static final double ELEVATOR_TICKS_PER_INCH = 1716;
    private static final double ENCODER_TOLERANCE = 20;

    private ElapsedTime timer = new ElapsedTime();

    private PIDCoefficients PIDcoeffs;
    private org.firstinspires.ftc.teamcode.lib.drive.PIDController PIDController;

    //Private constructor

    private ElevatingArm() {
        //yeet
    }

    @Override
    public void subsystemInit(HardwareMap hardwareMap) {
        //Init arm motors
        elevatorArmLeft = hardwareMap.get(DcMotor.class, "elevatorArmLeft");
        elevatorArmRight = hardwareMap.get(DcMotor.class, "elevatorArmRight");
        rotationalArm = hardwareMap.get(DcMotor.class, "rotationalArm");

        inSensor = hardwareMap.get(DigitalChannel.class, "inSensor");
        inSensor.setMode(DigitalChannel.Mode.INPUT);

        //Set arm motors to break
        elevatorArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Enable encoders
        elevatorArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationalArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reverse left elevator motor
        elevatorArmLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Init PID members
        PIDcoeffs = new PIDCoefficients(0, 0, 0);
        DcMotor[] motors = {rotationalArm};
        PIDController = new PIDController(motors, PIDcoeffs, 50);


    }

    @Override
    public void teleopTick() {
        //Elevator arm controls
        if (RobotMain.gamepad1.dpad_up || RobotMain.gamepad2.dpad_up) {
            driveElevatorArm(VIDIPT_ELEVATOR_CONTROL);
        } else if (RobotMain.gamepad1.dpad_down || RobotMain.gamepad2.dpad_down) {
            driveElevatorArm(-VIDIPT_ELEVATOR_CONTROL);
        } else {
            driveElevatorArm(0);
        }

        //Rotational arm controls
        driveRotationalArm(RobotMain.gamepad2.right_stick_y * VIDIPT_ROTATIONAL_ARM_CONTROL);
    }

    /**
     * @param power requested power to drive elevator
     */
    public void driveElevatorArm(double power) {
        elevatorArmLeft.setPower(power * VIDIPT_ELEVATOR_CONTROL);
        elevatorArmRight.setPower(power * VIDIPT_ELEVATOR_CONTROL);
    }

    public void elevateArm(double power, double inches) {
        setElevatorEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int setPoint = toTicks(inches);

        elevatorArmLeft.setTargetPosition(setPoint);
        elevatorArmRight.setTargetPosition(setPoint);

        setElevatorEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotationalArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void rotateArmByTime(double power, long millis, boolean PID) {
        rotationalArm.setPower(power);
        delay(millis);
        rotationalArm.setPower(0);
    }

    public void rotateArm(double power, double ticks, boolean PID) {

        rotationalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int setPoint = (int) ticks;

        rotationalArm.setPower(power);

        rotationalArm.setTargetPosition(setPoint);
        rotationalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION); //this might also be what messes stuff up


        boolean[] reverse = new boolean[1];
        reverse[0] = false;

        if (PID) {
            PIDController.drive(setPoint, ENCODER_TOLERANCE, reverse);
        } else {
            rotationalArm.setPower(power);
            while (rotationalArm.isBusy()) {
                if (killArm(power)) {
                    rotationalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    return;
                }
            }
        }
    }

    public void rotateArm(double power, double ticks, boolean PID, Telemetry telemetry) {

        rotationalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int setPoint = (int) ticks;

        rotationalArm.setPower(power);

        killArm(power);

        rotationalArm.setTargetPosition(setPoint);
        rotationalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean[] reverse = new boolean[1];
        reverse[0] = false;

        if (PID) {
            PIDController.drive(setPoint, ENCODER_TOLERANCE, reverse);
        } else {
            rotationalArm.setPower(power);
            while (rotationalArm.isBusy()) {
                telemetry.addData("current position:", rotationalArm.getCurrentPosition());
                telemetry.update();
                if (killArm(power)) {
                    rotationalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    return;
                }
            }
            rotationalArm.setPower(0);
        }
    }

    public void rotateArmTest(double power, Telemetry telemetry) {
        driveRotationalArm(power);
        telemetry.addData("left", rotationalArm.getCurrentPosition());
        telemetry.update();
    }

    public void delay(long millis) {
        timer.reset();
        while (timer.milliseconds() < millis) {
            //yeet 2.0
        }
    }

    public void setElevatorEncoderMode(DcMotor.RunMode mode) {
        elevatorArmLeft.setMode(mode);
        elevatorArmRight.setMode(mode);
    }

// when arm is going towards a sensor and the sensor is touched, arm stops moving in that direction
    public boolean killArm(double power) {
        if (inSensor.getState() == false && power > 0) {
            rotationalArm.setPower(0);
            return true;
        } else {
            return false;

        } /*else if (outSensor.isPressed() && power < 0) {
            rotationalArm.setPower(0);
            return true;
        } */

    }

    /**
     * @param power requested power to drive rotational arm
     */
    public void driveRotationalArm(double power) {
        killArm(power);
        if (killArm(power)) {
            //finessed
        } else {
            rotationalArm.setPower(power * VIDIPT_ROTATIONAL_ARM_CONTROL);
       }
    }

    public int toTicks(double inches) {
        return (int) (ELEVATOR_TICKS_PER_INCH * inches);
    }

    /**
     * @return the singleton instance of the subsystem
     */
    public static Subsystem getInstance() {
        return instance;
    }
}
