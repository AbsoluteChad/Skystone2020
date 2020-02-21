package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue_loading;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTasks;
import org.firstinspires.ftc.teamcode.subsystems.Capstone;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatingArm;
import org.firstinspires.ftc.teamcode.subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

@Autonomous(name="BlueLoadFar", group="Linear Opmode")
public class BlueLoadFar extends LinearOpMode {

    private RobotMain robot;
    private DriveTrain driveTrain;
    private FoundationMover foundationMover;
    private ElevatingArm elevatingArm;
    private Gripper gripper;
    private Capstone capstone;

    private static int BLOCK_WIDTH = 10;
    private static int STRAFE_DIS_TO_FOUNDATION = 104;


    @Override
    public void runOpMode() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "blue", true);
        driveTrain = (DriveTrain) RobotMain.driveTrain;
        foundationMover = (FoundationMover) RobotMain.foundationMover;
        elevatingArm = (ElevatingArm) RobotMain.elevatingArm;
        gripper = (Gripper) RobotMain.gripper;
        capstone = (Capstone) RobotMain.capstone;

        capstone.reset();
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
            if (skystonePos == 'L') {
                driveTrain.driveDistance(0.7, 2 * Constants.BLOCK_WIDTH, 0, false);
                disToFoundation = Constants.STRAFE_DIS_TO_FOUNDATION + Constants.BLOCK_WIDTH ;
            } else if (skystonePos == 'M') {
                driveTrain.driveDistance(0.7, Constants.BLOCK_WIDTH, 0, false);
                disToFoundation = Constants.STRAFE_DIS_TO_FOUNDATION;
            } else if (skystonePos == 'R' || skystonePos == 'N') {
                disToFoundation = Constants.STRAFE_DIS_TO_FOUNDATION - Constants.BLOCK_WIDTH ;
            }


            elevatingArm.rotateArm(-0.7, Constants.ARM_OUT_TICKS, false, telemetry);
            gripper.autoSucc(-1, 700);

            //AutonomousTasks.parallelDriveAndArm(.8,disToFoundation,0,.7,2800,telemetry);
            elevatingArm.rotateArm(0.7, Constants.ARM_IN_TICKS, false);


            driveTrain.driveDistance(1, disToFoundation,180, false);
            AutonomousTasks.parallelDriveAndArm(1,15,90,-0.7, Constants.ARM_OUT_TICKS_2, telemetry);
            //driveTrain.driveDistance(1, 14, 90, false);
            //elevatingArm.rotateArm(0.7, -2400, false);

            //working stuff
            gripper.autoSucc(1, 700);
            elevatingArm.rotateArm(0.7, Constants.ARM_IN_TICKS, false);
            //elevatingArm.rotationalArm.setPower(0.1);
            telemetry.addData("checkpoint", 1);
            telemetry.update();
            foundationMover.lockFoundation();
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (timer.milliseconds() < 250) {
                // finessed
            }

            AutonomousTasks.parallelDriveAndArm(1,26,270,0.5, Constants.ARM_IN_TICKS_2, telemetry);
            //driveTrain.driveDistance(0.7, 18, 270, false);

            timer.reset();
            while (timer.milliseconds() < 2000) {
                driveTrain.driveTank(-1, 0);
            }

            driveTrain.driveDistance(1, 15, 90, false);
            foundationMover.unlockFoundation();
            timer.reset();
            while (timer.milliseconds() < 250) {
                // finessed
            }
            driveTrain.driveMecanum(1, 315, 2100);
            driveTrain.driveDistance(1, 15, 270, false);
        }
    }
}
