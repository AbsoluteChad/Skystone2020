package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue_building;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatingArm;
import org.firstinspires.ftc.teamcode.subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

@Autonomous(name="BlueBuildParkClose", group="Linear Opmode")
public class BlueBuildParkClose extends LinearOpMode {
    private RobotMain robot;
    private DriveTrain driveTrain;
    private FoundationMover foundationMover;

    @Override
    public void runOpMode() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "blue", true);
        driveTrain = (DriveTrain) RobotMain.driveTrain;
        foundationMover = (FoundationMover) RobotMain.foundationMover;

        foundationMover.unlockFoundation();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            driveTrain.driveDistance(0.5, 6, 90, false);
            driveTrain.driveDistance(0.5, 14, 0, false);
        }
    }
}

