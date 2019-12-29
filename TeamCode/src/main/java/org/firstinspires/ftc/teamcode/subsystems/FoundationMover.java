package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotMain;

public class FoundationMover extends Subsystem {

    //Declare instance
    private static final Subsystem instance = new FoundationMover();

    //Declare servos
    public Servo foundationLeft;
    public Servo foundationRight;

    //Private constructor
    private FoundationMover() {

    }

    @Override
    public void subsystemInit(HardwareMap hardwareMap) {
        foundationLeft = hardwareMap.get(Servo.class, "foundationLeft");
        foundationRight = hardwareMap.get(Servo.class, "foundationRight");

    }

    @Override
    public void teleopTick() {
        if (RobotMain.gamepad1.left_bumper) {
            unlockFoundation();
        } else if (RobotMain.gamepad1.right_bumper) {
            lockFoundation();
        }
    }

    public void lockFoundation(){
        foundationLeft.setPosition(0.5);
        foundationRight.setPosition(0.5);
    }

    public void unlockFoundation() {
        foundationLeft.setPosition(0);
        foundationRight.setPosition(0);
    }



    /**
     * @return the singleton instance of the subsystem
     */
    public static final Subsystem getInstance() {
        return instance;
    }
}
