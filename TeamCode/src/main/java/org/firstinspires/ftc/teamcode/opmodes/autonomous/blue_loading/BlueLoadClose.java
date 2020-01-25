package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue_loading;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTasks;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatingArm;
import org.firstinspires.ftc.teamcode.subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

@Autonomous(name="BlueLoadClose", group="Linear Opmode")
public class BlueLoadClose extends LinearOpMode {

    private RobotMain robot;
    private DriveTrain driveTrain;
    private FoundationMover foundationMover;
    private ElevatingArm elevatingArm;
    private Gripper gripper;

    private static int BLOCK_WIDTH = 10;
    private static int STRAFE_DIS_TO_FOUNDATION = 104;

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

        int disToFoundation = 0; //this variable used for calculating distance to foundation

        waitForStart();

        if (opModeIsActive()) {
            //Go forward and sense
            //elevatingArm.rotationalArm.setPower(-0.2);
            driveTrain.driveDistance(1, 24, 90, false);
            //elevatingArm.rotationalArm.setPower(0);

            String skystonePosition = "center"; /* robot.tensorFlow.getSkystonePosition(true, 5000);
            telemetry.addData("Position", skystonePosition);
            telemetry.update(); */


            if ((skystonePosition.equals("center")) || (skystonePosition.equals("nope"))){
                disToFoundation = STRAFE_DIS_TO_FOUNDATION;
            } else if (skystonePosition.equals("left")){
                disToFoundation = STRAFE_DIS_TO_FOUNDATION - BLOCK_WIDTH ;
                driveTrain.driveDistance(1, BLOCK_WIDTH, 0, false);
            } else if (skystonePosition.equals("right")) {
                disToFoundation = STRAFE_DIS_TO_FOUNDATION + BLOCK_WIDTH ;
                driveTrain.driveDistance(1, BLOCK_WIDTH, 180, false);
            }

            elevatingArm.rotateArm(0.7, -3000, false);
            gripper.autoSucc(-1, 1000);
            telemetry.addData("checkpoint", 1);
            telemetry.update();
            elevatingArm.rotateArm(0.7, 2700, false);

            driveTrain.driveDistance(1, disToFoundation,180, false);
            //driveTrain.driveDistance(1, 14, 90, false);
            //elevatingArm.rotateArm(0.7, -2400, false);

            AutonomousTasks.parallelDriveAndArm(0.7,14,90,.7,-2400,telemetry);
            gripper.autoSucc(1, 1000);
            //elevatingArm.rotateArm(0.7, 2400, false);
            //elevatingArm.rotationalArm.setPower(0.1);
            telemetry.addData("checkpoint", 1);
            telemetry.update();
            foundationMover.lockFoundation();

            AutonomousTasks.parallelDriveAndArm(1,19,270,.7,2400,telemetry);
            //driveTrain.driveDistance(0.7, 18, 270, false);
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (timer.milliseconds() < 3000) {
                driveTrain.driveTank(-1, 0);
            }
            driveTrain.driveDistance(0.7, 12, 90, false);
            foundationMover.unlockFoundation();
            driveTrain.driveDistance(1, 46, 270, false);
        }
    }
}
