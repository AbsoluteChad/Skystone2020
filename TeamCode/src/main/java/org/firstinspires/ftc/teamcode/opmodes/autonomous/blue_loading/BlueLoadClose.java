package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue_loading;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTasks;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatingArm;
import org.firstinspires.ftc.teamcode.subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Capstone;

@Autonomous(name="BlueLoadClose", group="Linear Opmode")
//@Disabled
public class BlueLoadClose extends LinearOpMode {

    private RobotMain robot;
    private DriveTrain driveTrain;
    private FoundationMover foundationMover;
    private ElevatingArm elevatingArm;
    private Gripper gripper;
    private Capstone capstone;



    @Override
    public void runOpMode() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "blue", true);
        driveTrain = (DriveTrain) RobotMain.driveTrain;
        foundationMover = (FoundationMover) RobotMain.foundationMover;
        elevatingArm = (ElevatingArm) RobotMain.elevatingArm;
        gripper = (Gripper) RobotMain.gripper;
        capstone = (Capstone) RobotMain.capstone;

        capstone.reset();

        telemetry.addData("Skystone position", robot.skystonePosition);

        foundationMover.unlockFoundation();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int disToFoundation = 0; //this variable used for calculating distance to foundation

        waitForStart();

        if (opModeIsActive()) {
            char skystonePos = robot.skystonePosition;
            driveTrain.driveDistance(1, 23, 90, false);
            telemetry.addData("skystone pos", skystonePos);
            telemetry.update();
            if (skystonePos == 'R') {
                driveTrain.driveDistance(0.7, 2, 0, false);
                disToFoundation = Constants.STRAFE_DIS_TO_FOUNDATION +  Constants.BLOCK_WIDTH -5;
            } else if (skystonePos == 'M') {
                driveTrain.driveDistance(0.7, Constants.BLOCK_WIDTH, 180, false);
                disToFoundation = Constants.STRAFE_DIS_TO_FOUNDATION - 5;
            } else if (skystonePos == 'L' || skystonePos == 'N') {
                driveTrain.driveDistance(0.7, 2 * Constants.BLOCK_WIDTH, 180, false);
                disToFoundation = Constants.STRAFE_DIS_TO_FOUNDATION -  Constants.BLOCK_WIDTH -5;
            }

            elevatingArm.rotateArm(-0.7, Constants.ARM_OUT_TICKS, false, telemetry);
            gripper.autoSucc(-1, 1200);

            //AutonomousTasks.parallelDriveAndArm(.8,disToFoundation,0,.7,2800,telemetry);
            elevatingArm.rotateArm(0.7, Constants.ARM_IN_TICKS, false);

            driveTrain.driveDistance(1, disToFoundation,180, false);
            AutonomousTasks.parallelDriveAndArm(0.7,14,90,-0.7, Constants.ARM_OUT_TICKS_2 + 200, telemetry);
            //driveTrain.driveDistance(1, 14, 90, false);
            //elevatingArm.rotateArm(0.7, -2400, false);
            gripper.autoSucc(1, 1000);

            telemetry.addData("checkpoint", 1);
            telemetry.update();
            foundationMover.lockFoundation();
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (timer.milliseconds() < 250) {
                // finessed
            }

            AutonomousTasks.parallelDriveAndArm(0.7,26,270,0.7, Constants.ARM_IN_TICKS_2, telemetry);

            timer.reset();
            while (timer.milliseconds() < 3000) {
                driveTrain.driveTank(-0.7, 0);
            }
            driveTrain.driveDistance(0.7, 15, 90, false);
            foundationMover.unlockFoundation();
            timer.reset();
            while (timer.milliseconds() < 250) {
                // finessed
            }
            driveTrain.driveDistance(1, 40, 270, false);
            timer.reset();
            while (timer.milliseconds() < 1000) {
                // finessed
            }

        }
    }
}
