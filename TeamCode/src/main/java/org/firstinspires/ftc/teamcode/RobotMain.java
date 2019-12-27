//test commit

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.*;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class RobotMain {

    //Declare hardwareMap
    public static HardwareMap hardwareMap;

    //Declare gamepads
    public static Gamepad gamepad1;
    public static Gamepad gamepad2;

    //Declare all & put in ArrayList
    public static Subsystem driveTrain = DriveTrain.getInstance();
    public static Subsystem elevatingArm = ElevatingArm.getInstance();
    public static Subsystem gripper = Gripper.getInstance();
    public static Subsystem[] allSubsystems = {driveTrain, elevatingArm, gripper};

    //Declare eemuu
    public static BNO055IMU gyro;

    //Miscellaneous
    public static final String VUFORIA_KEY = "Aa4qojf/////AAABmUtRp+oA10Tyg9NdvwIzzH4eVE09jioK/9lv2fPHeJLN4mXBj/AfGpZM/0ym7+uvZfeSNpIhhU3UJ" +
            "tFl9JRatjump2m7urI4tq+M1FtU/sEdTD4uHJjGuoI4HW7BTvLvxNxuEQZ3f3sexDW8F8FJPOkkJHnbycwT1m+h7EQqjnwiySsMWeKoN/Fu2cGljvuZ5LAGpVVosB2" +
            "plP1dtviSkJbGy7MsHmJjL/NqEv/fjuiFqlra9Y29n8oZRoDsvwJkHJw/oQIv4kpTRHMSKV6NZZeyRm46zsb7mFkW0yXDpANgWqCjvAJVPm5W5JTcq8IZqDUn0bJyQ" +
            "Ju/F0OFbg2JsKHBcxNA1hasVyUxPrlIQCYc";

    public RobotMain(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        robotInit();
    }

    private void robotInit() {
        //Init all subsystems
        for (Subsystem subsystem : allSubsystems) {
            subsystem.subsystemInit(hardwareMap);
        }

        //Init eemuu
        gyro = (BNO055IMU) hardwareMap.get(Gyroscope.class, "imu");
    }

    //For if regular framework stops working
    public DcMotor getDcMotor(String name) { return hardwareMap.get(DcMotor.class, name); }

    public Servo getServoMotor(String name) { return hardwareMap.get(Servo.class, name); }

    public CRServo getCRServoMotor(String name) { return hardwareMap.get(CRServo.class, name); }

    //Gyro control
    public static double getAngle() {
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
}
