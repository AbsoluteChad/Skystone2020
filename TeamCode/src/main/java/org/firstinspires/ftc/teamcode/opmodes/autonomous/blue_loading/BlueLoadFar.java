package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue_loading;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTasks;
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
            driveTrain.driveDistance(1, 22, 90, false);
            //elevatingArm.rotationalArm.setPower(0);

            String skystonePosition = "center"; /* robot.tensorFlow.getSkystonePosition(true, 5000);
            telemetry.addData("Position", skystonePosition);
            telemetry.update();
            driveTrain.driveDistance(1, 11, 90, false); */

            if ((skystonePosition.equals("center")) || (skystonePosition.equals("nope"))){
                disToFoundation = Constants.STRAFE_DIS_TO_FOUNDATION;
            } else if (skystonePosition.equals("left")){
                disToFoundation = Constants.STRAFE_DIS_TO_FOUNDATION + Constants.BLOCK_WIDTH;
                driveTrain.driveDistance(1, Constants.BLOCK_WIDTH, 180, false);
            } else if (skystonePosition.equals("right")) {
                disToFoundation = Constants.STRAFE_DIS_TO_FOUNDATION - Constants.BLOCK_WIDTH ;
                driveTrain.driveDistance(1, Constants.BLOCK_WIDTH, 0, false);
            }

            elevatingArm.rotateArm(-0.7, Constants.ARM_OUT_TICKS, false);
            gripper.autoSucc(-1, 1000);
            telemetry.addData("checkpoint", 1);
            telemetry.update();
            //AutonomousTasks.parallelDriveAndArm(.8,disToFoundation,0,.7,2800,telemetry);
            elevatingArm.rotateArm(0.7, Constants.ARM_IN_TICKS, false);


            driveTrain.driveDistance(1, disToFoundation,180, false);
            AutonomousTasks.parallelDriveAndArm(0.7,18,90,-.7,Constants.ARM_OUT_TICKS_2, telemetry);
            //driveTrain.driveDistance(1, 14, 90, false);
            //elevatingArm.rotateArm(0.7, -2400, false);

            //working stuff
            gripper.autoSucc(1, 1000);
            elevatingArm.rotateArm(0.7, Constants.ARM_IN_TICKS, false);
            //elevatingArm.rotationalArm.setPower(0.1);
            telemetry.addData("checkpoint", 1);
            telemetry.update();
            foundationMover.lockFoundation();

            AutonomousTasks.parallelDriveAndArm(1,20,270,0.5, Constants.ARM_IN_TICKS_2, telemetry);
            //driveTrain.driveDistance(0.7, 18, 270, false);
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (timer.milliseconds() < 2500) {
                driveTrain.driveTank(-1, 0);
            }
            driveTrain.driveDistance(1, 20, 90, false);
            foundationMover.unlockFoundation();
            timer.reset();
            while (timer.milliseconds() < 500) {
                // finessed
            }
            driveTrain.driveMecanum(1, 315, 3000);
            driveTrain.driveDistance(1, 13, 270, false);
        }
    }
}
