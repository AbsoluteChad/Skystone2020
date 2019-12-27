package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotMain;

public class Gripper extends Subsystem {

    //Declare private instance
    private static final Subsystem instance = new Gripper();

    //Declare CRServos
    //public CRServo gripperClose;
    //public CRServo gripperFar;

    //Declare constants
    private final double GRIPPER_SUCC_POWER = 0.5;

    //Private constructor
    private Gripper() {

    }

    @Override
    public void subsystemInit(HardwareMap hardwareMap) {
        //Init CRServos
        //gripperClose = hardwareMap.get(CRServo.class, "gripperClose");
        //gripperFar = hardwareMap.get(CRServo.class, "gripperFar");
    }

    @Override
    public void teleopTick() {
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
    }

    /**
     * Controls compliant wheel intake
     * @param gripperClosePower power applied to close compliant wheel
     * @param gripperFarPower power applied to far compliant wheel
     */
    public void gripperSucc(double gripperClosePower, double gripperFarPower) {
        //gripperClose.setPower(gripperClosePower);
        //gripperFar.setPower(-gripperFarPower);
    }

    /**
     * @return the singleton instance of the subsystem
     */
    public static Subsystem getInstance() {
        return instance;
    }
}
