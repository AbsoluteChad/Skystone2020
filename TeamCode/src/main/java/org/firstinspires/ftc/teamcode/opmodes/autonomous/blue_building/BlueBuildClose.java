package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue_building;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.Capstone;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatingArm;
import org.firstinspires.ftc.teamcode.subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

@Autonomous(name="BlueBuildClose", group="Linear Opmode")
//@Disabled
public class BlueBuildClose extends LinearOpMode {
    private RobotMain robot;
    private DriveTrain driveTrain;
    private FoundationMover foundationMover;
    private ElevatingArm elevatingArm;
    private Gripper gripper;
    private Capstone capstone;

    @Override
    public void runOpMode() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "blue", false);
        driveTrain = (DriveTrain) RobotMain.driveTrain;
        foundationMover = (FoundationMover) RobotMain.foundationMover;
        elevatingArm = (ElevatingArm) RobotMain.elevatingArm;
        gripper = (Gripper) RobotMain.gripper;
        capstone = (Capstone) RobotMain.capstone;

        capstone.reset();

        foundationMover.unlockFoundation();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            //String skystonePos = robot.getSkystonePosition(false, 0);
            //driveTrain.driveDistance(0.7, 12, 90, false);

            driveTrain.driveMecanum(0.7, 135, 1800); //traveling diagonal in order to position with the foundation
            driveTrain.driveDistance(0.7, 19, 90, false); //moving straight in order to get close enough to the foundation
            foundationMover.lockFoundation(); //locking onto Foundation
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (timer.milliseconds() < 250) {
                // finessed
            }
            driveTrain.driveDistance(0.7, 26, 270, false);
            timer.reset();
            while (timer.milliseconds() < 3000) {
                driveTrain.driveTank(-0.7, 0);
            }
            driveTrain.driveDistance(0.7, 14, 90, false);
            foundationMover.unlockFoundation();
            timer.reset();
            while (timer.milliseconds() < 250) {
                // finessed
            }
            driveTrain.driveDistance(0.7, 45, 270, false);

            timer.reset();
            while (timer.milliseconds() < 2000) {
                // finessed
            }

        }
    }
}

