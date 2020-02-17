package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

public class Capstone extends Subsystem {

    //Declare instance
    private static final Subsystem instance = new org.firstinspires.ftc.teamcode.subsystems.Capstone();

    //Declare servos
    public Servo dispenser;

    //Private constructor
    private Capstone() {

    }

    @Override
    public void subsystemInit(HardwareMap hardwareMap) {
        dispenser = hardwareMap.get(Servo.class, "gripperRotation");
    }

    @Override
    public void teleopTick() {
        if (RobotMain.gamepad2.a) {
            dispenser.setPosition(1);
            dispenser.setPosition(0);
        }
    }

    /**
     * @return the singleton instance of the subsystem
     */
    public static final Subsystem getInstance() {
        return instance;
    }
}
