package org.firstinspires.ftc.teamcode.opmodes.autonomous.red_building;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.Capstone;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatingArm;
import org.firstinspires.ftc.teamcode.subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

@Autonomous(name="RedBuildParkFar", group="Linear Opmode")
public class RedBuildParkFar extends LinearOpMode {
    private RobotMain robot;
    private DriveTrain driveTrain;
    private FoundationMover foundationMover;
    private Capstone capstone;


    @Override
    public void runOpMode() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "red", false);
        driveTrain = (DriveTrain) RobotMain.driveTrain;
        foundationMover = (FoundationMover) RobotMain.foundationMover;
        capstone = (Capstone) RobotMain.capstone;
        capstone.reset();

        foundationMover.unlockFoundation();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            driveTrain.driveDistance(0.5, 28, 90, false);
            driveTrain.driveDistance(0.5, 20, 180, false);
        }
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < 2000) {
            // finessed
        }

    }
}

