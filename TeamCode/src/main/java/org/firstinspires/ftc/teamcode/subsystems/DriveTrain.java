package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;

public class DriveTrain extends Subsystem {

    //Declare private instance
    private static final Subsystem instance = new DriveTrain();

    //Declare motors
    /*public DcMotor topLeft;
    public DcMotor bottomLeft;
    public DcMotor topRight;
    public DcMotor bottomRight;*/

    //Declare constants
    private static final double VIDIPT_DRIVE_CONTROL = 1;
    private static final double WHEEL_CIRCUMFRENCE = 4 * Math.PI;
    private static final int TICKS_PER_ROTATION = 1440;
    private static final double GEAR_RATIO = 1;

    //Declare PID members
    private ElapsedTime timer = new ElapsedTime();
    private final double LOOP_TIME = 0.02;

    private double error;
    private double prevError;

    private double P, I, D;
    private double kP, kI, kD;

    //Private constructor
    private DriveTrain() {
        //Init PID members
        kP = 0;
        kI = 0;
        kD = 0;
        prevError = 0;
    }

    @Override
    public void subsystemInit(HardwareMap hardwareMap) {
        //Init motors
        /*topLeft = hardwareMap.get(DcMotor.class, "topLeft");
        bottomLeft = hardwareMap.get(DcMotor.class, "bottomLeft");
        topRight = hardwareMap.get(DcMotor.class, "topRight");
        bottomRight = hardwareMap.get(DcMotor.class, "bottomRight");

        //Init eemuu
        gyro = (BNO055IMU) hardwareMap.get(Gyroscope.class, "imu");

        //Set motors to break
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reverse left side motors
        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Enable encoders
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
    }

    @Override
    public void teleopTick() {
        double thrust = -RobotMain.gamepad1.left_stick_y;
        double strafe = RobotMain.gamepad1.left_stick_x;
        double turn = RobotMain.gamepad1.right_stick_x;
        driveMecanum(thrust, strafe, turn, false);
    }

    /**
     * Basic tank drive that controls left wheels and right wheels independently
     *
     * @param leftPower power applied to left wheels
     * @param rightPower power applied to right wheels
     */
    public void driveTank(double leftPower, double rightPower) {
        /*topLeft.setPower(leftPower * VIDIPT_DRIVE_CONTROL);
        bottomLeft.setPower(leftPower * VIDIPT_DRIVE_CONTROL);
        topRight.setPower(rightPower * VIDIPT_DRIVE_CONTROL);
        bottomRight.setPower(rightPower * VIDIPT_DRIVE_CONTROL);*/
    }

    /**
     * Determines mecanum wheel powers with given components. Feed in joystick values to drive in
     * desired direction.
     *
     * @param thrust y component of requested vector
     * @param strafe x component of requested vector
     * @param turn turn component (z axis) of robot while traveling along requested vector
     * @param fieldCentric whether robot should move relative to field or drivers
     *                     (true = relative to field; false = relative to drivers)
     */
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
        /*topLeft.setPower(topLeftPower);
        bottomLeft.setPower(bottomLeftPower);
        topRight.setPower(topRightPower);
        bottomRight.setPower(bottomRightPower);*/
    }

    /**
     * Feeds mecanum wheel powers based off an inputted direction
     *
     * @param degreeDirection the unit circle direction (degrees) requested to drive
     */
    public void driveMecanum(double degreeDirection) {
        double radianDirection = Math.toRadians(degreeDirection);
        double thrust = Math.sin(radianDirection);
        double strafe = Math.cos(radianDirection);
        driveMecanum(thrust, strafe, 0, false);
    }

    /**
     * Drives a certain distance using encoders
     *
     * @param power power applied to all motors
     * @param leftInches distance traveled by left wheels of drivetrain
     * @param rightInches distance traveled by right wheels of drivetrain
     */
    public void driveDistance(double power, int leftInches, int rightInches) {
        /*int leftTicks = toTicks(leftInches);
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
            //Yeet
        }
        driveTank(0, 0);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
    }

    /**
     * Uses a PID controller to accurately ramp up and ramp down power to reach an end position
     * <i>setPoint</i> inches away
     *
     * @param setPoint desired robot end position
     */
    public void drivePID(double setPoint) {
        /*setPoint = toTicks(setPoint);
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
        bottomRight.setPower(0);*/
    }

    /**
     * Used to rotate a certain number of degrees on unit circle
     *
     * @param degrees degree amount to turn
     *                negative value = clockwise (about unit circle)
     *                positive value = counterclockwise
     * @param PID whether or not to use PID
     */
    public void rotateDegrees(double power, double degrees, boolean PID) {
        //Convert degrees in terms of absolute angle
        /*double requestedAngle = getAngle() + degrees;

        //Ensure that requested angle is "in bounds"
        if (requestedAngle < 0) {
            requestedAngle += 360;
        } else if (requestedAngle >= 360) {
            requestedAngle -= 360;
        }

        //Call rotateTo() and pass in absolute angle
        rotateTo(power, requestedAngle, PID);*/
    }

    /**
     * Used to rotate to a certain degree position on unit circle
     *
     * @param degrees degree value to turn to
     *                negative value = clockwise
     *                positive value = counterclockwise
     * @param PID whether or not to use PID
     */
    //TODO finish method
    public void rotateTo(double power, double degrees, boolean PID) {
        if (PID) {

        } else {

        }
    }

    /**
     * @param inches distance in inches
     * @return <i>inches</i> converted to encoder ticks
     */
    private int toTicks(double inches) {
        double rotations = inches / WHEEL_CIRCUMFRENCE;
        return (int) (rotations * TICKS_PER_ROTATION / GEAR_RATIO);
    }

    /**
     * @return the singleton instance of the subsystem
     */
    public static Subsystem getInstance() {
        return instance;
    }
}
