//test commit

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class RobotMain {

    //Declare hardwareMap
    private HardwareMap hardwareMap;

    //Declare drive motors
    public DcMotor topLeft;
    public DcMotor bottomLeft;
    public DcMotor topRight;
    public DcMotor bottomRight;

    //Declare arm motors
    public DcMotor elevatorArmLeft;
    public DcMotor elevatorArmRight;
    public DcMotor rotationalArm;

    //Declare servos
    public CRServo gripperClose;
    public CRServo gripperFar;

    //Declare eemuu
    public BNO055IMU gyro;

    //Constants
    private static final double WHEEL_CIRCUMFRENCE = 4 * Math.PI;
    private static final double GEAR_RATIO = 1;
    private static final int TICKS_PER_ROTATION = 1440;

    //Vidipt controls
    private static final double VIDIPT_DRIVE_CONTROL = 1;
    private static final double VIDIPT_ELEVATOR_ARM_CONTROL = 1;
    private static final double VIDIPT_ROTATIONAL_ARM_CONTROL = 0.5;

    //PID
    private ElapsedTime timer = new ElapsedTime();
    private final double LOOP_TIME = 0.02;

    private double error;
    private double prevError;

    private double P, I, D;
    private double kP, kI, kD;

    public RobotMain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        //Init drive motors
        topLeft = hardwareMap.get(DcMotor.class, "topLeft");
        bottomLeft = hardwareMap.get(DcMotor.class, "bottomLeft");
        topRight = hardwareMap.get(DcMotor.class, "topRight");
        bottomRight = hardwareMap.get(DcMotor.class, "bottomRight");

        //Init arm motors
        elevatorArmLeft = hardwareMap.get(DcMotor.class, "elevatorArmLeft");
        elevatorArmRight = hardwareMap.get(DcMotor.class, "elevatorArmRight");
        rotationalArm = hardwareMap.get(DcMotor.class, "rotationalArm");

        //Init servos
        gripperClose = hardwareMap.get(CRServo.class, "gripperClose");
        gripperFar = hardwareMap.get(CRServo.class, "gripperFar");

        //Init eemuu
        gyro = (BNO055IMU) hardwareMap.get(Gyroscope.class, "imu");

        //Set drive motors to break
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set arm motors to break
        elevatorArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reverse left side motors
        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorArmLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set using encoders
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //PID
        kP = 0;
        kI = 0;
        kD = 0;
        prevError = 0;
    }

    //For if regular framework stops working
    public DcMotor getDcMotor(String name) {
        return hardwareMap.get(DcMotor.class, name);
    }

    public Servo getServoMotor(String name) {
        return hardwareMap.get(Servo.class, name);
    }

    public CRServo getCRServoMotor(String name) {
        return hardwareMap.get(CRServo.class, name);
    }

    //PID
    public void drivePID(double setPoint) {
        setPoint = toTicks(setPoint);
        timer.reset();

        while (Math.abs(calcPIDError(topLeft, setPoint)) < 5 || timer.seconds() < 5) {
            //Calculate error
            error = calcPIDError(topLeft, setPoint);

            //P, I, and D w/o gains
            P = error;
            I += (error * LOOP_TIME);
            D = (error - prevError) / LOOP_TIME;

            //Implement gains
            P *= kP;
            I *= kI;
            D *= kD;

            //Calc & apply output
            double output = P + I + D;
            topLeft.setPower(output);
            bottomLeft.setPower(output);
            topRight.setPower(output);
            bottomRight.setPower(output);

            //Calculate prevError
            prevError = error;
        }

        topLeft.setPower(0);
        bottomLeft.setPower(0);
        topRight.setPower(0);
        bottomRight.setPower(0);
    }

    public double calcPIDError(DcMotor motor, double setPoint) {
        return setPoint - motor.getCurrentPosition();
    }

    //Non PID drive
    public void driveTank(double leftPower, double rightPower) {
        topLeft.setPower(leftPower * VIDIPT_DRIVE_CONTROL);
        bottomLeft.setPower(leftPower * VIDIPT_DRIVE_CONTROL);
        topRight.setPower(rightPower * VIDIPT_DRIVE_CONTROL);
        bottomRight.setPower(rightPower * VIDIPT_DRIVE_CONTROL);
    }

    public void driveMecanum(double thrust, double strafe, double turn, boolean fieldCentric) {
        //If field centric -- does nothing yet
        if (fieldCentric) {
            thrust = thrust;
            strafe = strafe;
        }

        //Determining wheel powers
        double topLeftPower = thrust + strafe + turn;
        double bottomLeftPower = thrust - strafe + turn;
        double topRightPower = thrust - strafe - turn;
        double bottomRightPower = thrust + strafe - turn;

        //Scale based off max power
        double maxPower = Math.max(Math.max(Math.max(topLeftPower, bottomLeftPower), topRightPower), bottomRightPower);
        if (Math.abs(maxPower) > 1.0) {
            topLeftPower /= Math.abs(maxPower);
            bottomLeftPower /= Math.abs(maxPower);
            topRightPower /= Math.abs(maxPower);
            bottomRightPower /= Math.abs(maxPower);
        }

        //Vidipt control
        topLeftPower *= VIDIPT_DRIVE_CONTROL;
        bottomLeftPower *= VIDIPT_DRIVE_CONTROL;
        topRightPower *= VIDIPT_DRIVE_CONTROL;
        bottomRightPower *= VIDIPT_DRIVE_CONTROL;

        //Set powers
        topLeft.setPower(topLeftPower);
        bottomLeft.setPower(bottomLeftPower);
        topRight.setPower(topRightPower);
        bottomRight.setPower(bottomRightPower);
    }

    public void driveMecanum(double power, double degreeDirection) {
        double radianDirection = Math.toRadians(degreeDirection);
        double thrust = Math.sin(radianDirection);
        double strafe = Math.cos(radianDirection);
        driveMecanum(thrust, strafe, 0, false);
    }

    public void driveDistance(double power, int leftInches, int rightInches) {
        int leftTicks = toTicks(rightInches);
        int rightTicks = toTicks(rightInches);

        topLeft.setTargetPosition(topLeft.getCurrentPosition() + leftTicks);
        bottomLeft.setTargetPosition(bottomLeft.getCurrentPosition() + leftTicks);
        topRight.setTargetPosition(topRight.getCurrentPosition() + rightTicks);
        bottomRight.setTargetPosition(bottomRight.getCurrentPosition() + rightTicks);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveTank(power, power);
        while (topLeft.isBusy() || bottomLeft.isBusy() || topRight.isBusy() || bottomRight.isBusy()) {
            //Empty while loop
        }
        driveTank(0, 0);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Gyro controls
    public double getAngle() {
        //Retrieve raw angle
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double rawAngle = angles.firstAngle;

        //Manipulate angle to unit circle convention (degrees)
        if (rawAngle < 0) {
            rawAngle = 360 + rawAngle;
        }

        //Orient to unit circle convention (degrees)
        rawAngle -= 90;
        if (rawAngle < 0) {
            rawAngle += 360;
        }

        return rawAngle;
    }

    //Arm control
    public void driveElevatorArm(double power) {
        elevatorArmLeft.setPower(power * VIDIPT_ELEVATOR_ARM_CONTROL);
        elevatorArmRight.setPower(power * VIDIPT_ELEVATOR_ARM_CONTROL);
    }

    public void driveRotationalArm(double power) {
        rotationalArm.setPower(power * VIDIPT_ROTATIONAL_ARM_CONTROL);
    }

    //CRServo gripper controls
    public void gripperSucc(double gripperClosePower, double gripperFarPower) {
        gripperClose.setPower(gripperClosePower);
        gripperFar.setPower(-gripperFarPower);
    }

    //Miscellaneous
    private int toTicks(double inches) {
        double rotations = inches / WHEEL_CIRCUMFRENCE;
        return (int) (rotations * TICKS_PER_ROTATION / GEAR_RATIO);
    }
}