package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Subsystem {

    public abstract void subsystemInit(HardwareMap hardwareMap);
    public abstract void teleopTick();
}
