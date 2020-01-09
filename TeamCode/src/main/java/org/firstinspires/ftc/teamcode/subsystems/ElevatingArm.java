package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMain;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ElevatingArm extends Subsystem {

    //Declare private instance
    private static final ElevatingArm instance = new ElevatingArm();

    //Declare motors
    public DcMotor elevatorArmLeft;
    public DcMotor elevatorArmRight;
    public DcMotor rotationalArm;

    //Declare constants
    private static final double VIDIPT_ELEVATOR_CONTROL = 0.75;
    private static final double VIDIPT_ROTATIONAL_ARM_CONTROL = 0.6;
    private static final double ELEVATOR_TICKS_PER_INCH = 1716;

    private ElapsedTime timer = new ElapsedTime();

    //Private constructor

    private ElevatingArm() {

    }

    @Override
    public void subsystemInit(HardwareMap hardwareMap) {
        //Init arm motors
        elevatorArmLeft = hardwareMap.get(DcMotor.class, "elevatorArmLeft");
        elevatorArmRight = hardwareMap.get(DcMotor.class, "elevatorArmRight");
        rotationalArm = hardwareMap.get(DcMotor.class, "rotationalArm");

        //Set arm motors to break
        elevatorArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Enable elevator encoders
        elevatorArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reverse left elevator motor
        elevatorArmLeft.setDirection(DcMotorSimple.Direction.REVERSE);
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

    //test
    public void eleTest(double power, Telemetry telemetry) {
        driveElevatorArm(power);
        telemetry.addData("left", elevatorArmLeft.getCurrentPosition());
        telemetry.addData("right", elevatorArmRight.getCurrentPosition());
        telemetry.update();
    }

    public void elevateArm(double power, double inches) {
        setElevatorEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int setPoint = toTicks(inches);

        elevatorArmLeft.setTargetPosition(setPoint);
        elevatorArmRight.setTargetPosition(setPoint);

        setElevatorEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void rotateArm(double power, long millis, boolean PID) {
        rotationalArm.setPower(power);
        delay(millis);
        rotationalArm.setPower(0);
    }

    public void delay(long millis) {
        timer.reset();
        while (timer.milliseconds() < millis) {
            //yeet 2.0
        }
    }

    public int toTicks(double inches) {
        return (int) (ELEVATOR_TICKS_PER_INCH * inches);

    }

    public void setElevatorEncoderMode(DcMotor.RunMode mode) {
        elevatorArmLeft.setMode(mode);
        elevatorArmRight.setMode(mode);
    }

    /**
     * @param power requested power to drive rotational arm
     */
    public void driveRotationalArm(double power) {
        rotationalArm.setPower(power * VIDIPT_ROTATIONAL_ARM_CONTROL);
    }

    /**
     * @return the singleton instance of the subsystem
     */
    public static Subsystem getInstance() {
        return instance;
    }
}
