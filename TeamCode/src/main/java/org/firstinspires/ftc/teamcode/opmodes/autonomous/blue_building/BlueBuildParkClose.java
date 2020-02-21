package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue_building;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.Capstone;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatingArm;
import org.firstinspires.ftc.teamcode.subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

@Autonomous(name="BlueBuildParkClose", group="Linear Opmode")
public class BlueBuildParkClose extends LinearOpMode {
    private RobotMain robot;
    private DriveTrain driveTrain;
    private FoundationMover foundationMover;
    private Capstone capstone;

    @Override
    public void runOpMode() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "blue", false);
        driveTrain = (DriveTrain) RobotMain.driveTrain;
        foundationMover = (FoundationMover) RobotMain.foundationMover;
        capstone = (Capstone) RobotMain.capstone;

        capstone.reset();
        foundationMover.unlockFoundation();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            driveTrain.driveDistance(0.5, 3, 90, false);
            driveTrain.driveDistance(0.5, 22, 0, false);
        }
    }
}

