package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotMain;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Gripper extends Subsystem {

    //Declare private instance
    private static final Subsystem instance = new Gripper();

    //Declare CRServos
    public CRServo gripperClose;
    public CRServo gripperFar;

    //Declare Servo
    public Servo gripperRotation;

    ElapsedTime timer = new ElapsedTime();



    //Declare constants
    private final double GRIPPER_SUCC_POWER = 0.5;
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position


    //Private constructor
    private Gripper() {

    }

    @Override
    public void subsystemInit(HardwareMap hardwareMap) {
        //Init CRServos
        gripperClose = hardwareMap.get(CRServo.class, "gripperClose");
        gripperFar = hardwareMap.get(CRServo.class, "gripperFar");

        //Init Servo
        gripperRotation = hardwareMap.get(Servo.class, "gripperRotation");

    }

    @Override
    public void teleopTick() {
        //CRServo controls
        double gripperClosePower;
        double gripperFarPower;
        if (RobotMain.gamepad1.y || RobotMain.gamepad2.y) {
            gripperClosePower = GRIPPER_SUCC_POWER;
            gripperFarPower = GRIPPER_SUCC_POWER;
        } else if (RobotMain.gamepad1.x || RobotMain.gamepad2.x) {
            gripperClosePower = -GRIPPER_SUCC_POWER;
            gripperFarPower = -GRIPPER_SUCC_POWER;
        } else {
            gripperClosePower = RobotMain.gamepad2.left_bumper ? -RobotMain.gamepad2.left_trigger : RobotMain.gamepad2.left_trigger;
            gripperFarPower = RobotMain.gamepad2.right_bumper ? -RobotMain.gamepad2.right_trigger : RobotMain.gamepad2.right_trigger;
        }
        gripperSucc(gripperClosePower, gripperFarPower);

        //Servo controls
        /*double gripperPosition = 0.5;
        if (RobotMain.gamepad1.a || RobotMain.gamepad2.a) {
            gripperPosition -= INCREMENT ;
            if (gripperPosition <= MIN_POS ) {
                gripperPosition = MIN_POS;
            }
        }
        if (RobotMain.gamepad1.b || RobotMain.gamepad2.b) {
            gripperPosition += INCREMENT;
            if (gripperPosition >= MAX_POS) {
                gripperPosition = MAX_POS;
            }
        }
        gripperRotate(gripperPosition);*/


    }


        //gripper rotation servo control

    /**
     * Controls compliant wheel intake
     * @param gripperClosePower power applied to close compliant wheel
     * @param gripperFarPower power applied to far compliant wheel
     */
    public void gripperSucc(double gripperClosePower, double gripperFarPower) {
        gripperClose.setPower(gripperClosePower);
        gripperFar.setPower(-gripperFarPower);
    }

    public void gripperRotate(double gripperPosition) {
        gripperRotation.setPosition(gripperPosition);
    }

    public void autoSucc(double gripperPower, long millis) {
        gripperClose.setPower(gripperPower);
        gripperFar.setPower(-gripperPower);
        delay(millis);
        gripperClose.setPower(0);
        gripperFar.setPower(0);
    }

    public void delay(long millis) {
        timer.reset();
        while (timer.milliseconds() < millis) {
            //yeet 2.0
        }
    }


    /**
     * @return the singleton instance of the subsystem
     */
    public static Subsystem getInstance() {
        return instance;
    }


}


