package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.lib.PIDController;

public class DriveTrain extends Subsystem {

    //Declare private instance
    private static final Subsystem instance = new DriveTrain();

    //Declare motors
    public DcMotor topLeft;
    public DcMotor bottomLeft;
    public DcMotor topRight;
    public DcMotor bottomRight;

    //Declare constants
    private static final double VIDIPT_DRIVE_CONTROL = 1;
    private static final double WHEEL_CIRCUMFRENCE = 4 * Math.PI;
    private static final int TICKS_PER_ROTATION = 1440;
    private static final double GEAR_RATIO = 1;
    private static final double ENCODER_ERROR = 5;

    //Declare PID members
    private PIDCoefficients PIDcoeffs;

    //Private constructor
    private DriveTrain() {
        //Init PID members
        PIDcoeffs = new PIDCoefficients(0, 0, 0);
    }

    @Override
    public void subsystemInit(HardwareMap hardwareMap) {
        //Init motors
        topLeft = hardwareMap.get(DcMotor.class, "topLeft");
        bottomLeft = hardwareMap.get(DcMotor.class, "bottomLeft");
        topRight = hardwareMap.get(DcMotor.class, "topRight");
        bottomRight = hardwareMap.get(DcMotor.class, "bottomRight");

        //Set motors to break
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reverse left side motors
        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Enable & reset encoders
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        topLeft.setPower(leftPower * VIDIPT_DRIVE_CONTROL);
        bottomLeft.setPower(leftPower * VIDIPT_DRIVE_CONTROL);
        topRight.setPower(rightPower * VIDIPT_DRIVE_CONTROL);
        bottomRight.setPower(rightPower * VIDIPT_DRIVE_CONTROL);
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
        topLeft.setPower(topLeftPower);
        bottomLeft.setPower(bottomLeftPower);
        topRight.setPower(topRightPower);
        bottomRight.setPower(bottomRightPower);
    }

    /**
     * Feeds mecanum wheel powers based off an inputted direction
     *
     * @param degreeDirection the unit circle direction (degrees) requested to drive
     */
    public void driveMecanum(double power, double degreeDirection, double inches, boolean PID, int yeet) {
        double radianDirection = Math.toRadians(degreeDirection);
        if (degreeDirection % 180 == 0) {
            int setPoint = toTicks(inches);
            setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //topLeft.setTargetPosition(Math.sin(radianDirection) > 0 ? 1 : -1 );
            //bottomLeft.setTargetPosition();
            //topRight.setTargetPosition();
            //bottomRight.setTargetPosition();
            setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (PID) {
                PIDController TLBR = new PIDController(PIDcoeffs, toTicks(inches));
                PIDController BLTR = new PIDController(PIDcoeffs, toTicks(inches));
                while (Math.abs(TLBR.getError()) > ENCODER_ERROR || Math.abs(BLTR.getError()) > ENCODER_ERROR) {
                    driveTank(TLBR.motorOutput(topLeft.getCurrentPosition()), BLTR.motorOutput(topRight.getCurrentPosition()));
                }
            } else {
                driveTank(power, power);
                while (topLeft.isBusy() || bottomLeft.isBusy() || topRight.isBusy() || bottomRight.isBusy()) {
                    //yeet
                }
            }
        }

        driveTank(0, 0);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double thrust = Math.sin(radianDirection) * power;
        double strafe = Math.cos(radianDirection) * power;
        driveMecanum(thrust, strafe, 0, false);
    }

    /**
     * Drives a certain distance using encoders
     *
     * @param power power applied to all motors
     * @param inches distance traveled by each wheel of drivetrain
     * @param PID whether or not to use PID
     */
    public void driveDistance(double power, int inches, boolean PID) {
        int setPoint = toTicks(inches);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setTargetPosition(setPoint);
        bottomLeft.setTargetPosition(setPoint);
        topRight.setTargetPosition(setPoint);
        bottomRight.setTargetPosition(setPoint);
        setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (PID) {
            PIDController left = new PIDController(PIDcoeffs, setPoint);
            PIDController right = new PIDController(PIDcoeffs, setPoint);
            while (Math.abs(left.getError()) > ENCODER_ERROR || Math.abs(right.getError()) > ENCODER_ERROR) {
                driveTank(left.motorOutput(topLeft.getCurrentPosition()), right.motorOutput(topRight.getCurrentPosition()));
            }
        } else {
            driveTank(power, power);
            while (topLeft.isBusy() || bottomLeft.isBusy() || topRight.isBusy() || bottomRight.isBusy()) {
                //yeet
            }
        }

        driveTank(0, 0);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Used to rotate a certain number of degrees about unit circle using encoders
     *
     * @param degrees degree amount to turn
     *                negative value = clockwise
     *                positive value = counterclockwise
     * @param PID whether or not to use PID
     */
    public void rotateDegrees(double power, double degrees, boolean PID) {
        int setPoint = (int) ((degrees / 360) * TICKS_PER_ROTATION);
        if (Math.abs(setPoint) < ENCODER_ERROR) {
            return;
        }
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setTargetPosition(-setPoint);
        bottomLeft.setTargetPosition(-setPoint);
        topRight.setTargetPosition(setPoint);
        bottomRight.setTargetPosition(setPoint);

        setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (PID) {
            PIDController left = new PIDController(PIDcoeffs, -setPoint);
            PIDController right = new PIDController(PIDcoeffs, setPoint);
            while (Math.abs(left.getError()) > ENCODER_ERROR || Math.abs(right.getError()) > ENCODER_ERROR) {
                if (setPoint < 0) {
                    driveTank(left.motorOutput(topLeft.getCurrentPosition()), -right.motorOutput(topRight.getCurrentPosition()));
                } else if (setPoint > 0) {
                    driveTank(-left.motorOutput(topLeft.getCurrentPosition()), right.motorOutput(topRight.getCurrentPosition()));
                }
            }
        } else {
            if (setPoint < 0) {
                driveTank(power, -power);
            } else if (setPoint > 0) {
                driveTank(-power, power);
            }
        }

        while (topLeft.isBusy() || bottomLeft.isBusy() || topRight.isBusy() || bottomRight.isBusy()) {
            //Yeet
        }
        driveTank(0, 0);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Used to set mode of all drive encoders
     *
     * @param mode mode for encoders to be set to
     */
    public void setEncoderMode(DcMotor.RunMode mode) {
        topLeft.setMode(mode);
        bottomLeft.setMode(mode);
        topRight.setMode(mode);
        bottomRight.setMode(mode);
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
