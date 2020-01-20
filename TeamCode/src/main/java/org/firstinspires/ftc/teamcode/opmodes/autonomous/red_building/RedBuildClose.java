package org.firstinspires.ftc.teamcode.opmodes.autonomous.red_building;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatingArm;
import org.firstinspires.ftc.teamcode.subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

@Autonomous(name="RedBuildClose", group="Linear Opmode")
public class RedBuildClose extends LinearOpMode {
    private RobotMain robot;
    private DriveTrain driveTrain;
    private FoundationMover foundationMover;
    private ElevatingArm elevatingArm;
    private Gripper gripper;

    @Override
    public void runOpMode() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "blue", true);
        driveTrain = (DriveTrain) RobotMain.driveTrain;
        foundationMover = (FoundationMover) RobotMain.foundationMover;
        elevatingArm = (ElevatingArm) RobotMain.elevatingArm;
        gripper = (Gripper) RobotMain.gripper;

        foundationMover.unlockFoundation();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            //String skystonePos = robot.getSkystonePosition(false, 0);

            //driveTrain.driveDistance(0.7, 12, 90, false);

            driveTrain.driveMecanum(0.7, 45, 1800); //traveling diagonal in order to position with the foundation
            driveTrain.driveDistance(0.7, 17, 90, false); //moving straight in order to get close enough to the foundation
            foundationMover.lockFoundation(); //locking onto Foundation
            driveTrain.driveDistance(0.7, 35, 270, false); //moving back in order to position the foundation to the site
            foundationMover.unlockFoundation(); //unlocking off of foundation
            driveTrain.driveDistance(0.7, 32, 180, false); //strafing to the left in order for clearance
            driveTrain.driveDistance(0.7, 20, 90, false); //moving straight to position to the foundation sideways
            driveTrain.driveDistance(0.7, 8, 0, false); // moving sideways in order to contact the foundation to the wall
            driveTrain.driveDistance(0.7, 6, 90, false); //moving straight to go to the outer area
            driveTrain.driveDistance(0.7, 28, 180, false); //going sideways to park
        }
    }
}

