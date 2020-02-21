/*
Copyright 2019 FIRST Tech Challenge Team 12923

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.opmodes.autonomous.red_loading;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.vision.SkystoneDetector;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTasks;
import org.firstinspires.ftc.teamcode.subsystems.Capstone;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatingArm;
import org.firstinspires.ftc.teamcode.subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous(name="RedLoadFar", group="Linear Opmode")
public class RedLoadClose extends LinearOpMode {

    private RobotMain robot;
    private DriveTrain driveTrain;
    private FoundationMover foundationMover;
    private ElevatingArm elevatingArm;
    private Gripper gripper;
    private Capstone capstone;
    private SkystoneDetector skystoneDetector;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "blue", true);

        driveTrain = (DriveTrain) RobotMain.driveTrain;
        foundationMover = (FoundationMover) RobotMain.foundationMover;
        elevatingArm = (ElevatingArm) RobotMain.elevatingArm;
        gripper = (Gripper) RobotMain.gripper;
        capstone = (Capstone) RobotMain.capstone;

        telemetry.addData("Skystone position", robot.skystonePosition);

        int disToFoundation = 0; //this variable is used for calculating distance to foundation
        foundationMover.unlockFoundation();
        capstone.reset();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            char skystonePos = robot.skystonePosition;
            driveTrain.driveDistance(1, 23, 90, false);
            telemetry.addData("skystone pos", skystonePos);
            telemetry.update();
            if (skystonePos == 'L') {
                driveTrain.driveDistance(0.7, 2 * Constants.BLOCK_WIDTH, 180, false);
                disToFoundation = Constants.STRAFE_DIS_TO_FOUNDATION + Constants.BLOCK_WIDTH ;
            } else if (skystonePos == 'M') {
                driveTrain.driveDistance(0.7, Constants.BLOCK_WIDTH, 180, false);
                disToFoundation = Constants.STRAFE_DIS_TO_FOUNDATION;
            } else if (skystonePos == 'R' || skystonePos == 'N') {
                disToFoundation = Constants.STRAFE_DIS_TO_FOUNDATION - Constants.BLOCK_WIDTH ;
            }


            elevatingArm.rotateArm(-0.7, Constants.ARM_OUT_TICKS, false, telemetry);
            gripper.autoSucc(-1, 700);

            //AutonomousTasks.parallelDriveAndArm(.8,disToFoundation,0,.7,2800,telemetry);
            elevatingArm.rotateArm(0.7, Constants.ARM_IN_TICKS, false);

            driveTrain.driveDistance(1, disToFoundation,0, false);
            AutonomousTasks.parallelDriveAndArm(0.7,15,90,-0.7, Constants.ARM_OUT_TICKS_2 + 300, telemetry);
            //driveTrain.driveDistance(1, 14, 90, false);
            //elevatingArm.rotateArm(0.7, -2400, false);

            gripper.autoSucc(1, 1000);
            //           elevatingArm.rotateArm(0.7, 2400, false);
            //           elevatingArm.rotationalArm.setPower(0.1);
            telemetry.addData("checkpoint", 1);
            telemetry.update();

            foundationMover.lockFoundation();
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (timer.milliseconds() < 250) {
                // finessed
            }

//          driveTrain.driveDistance(0.7, 19, 270, false);
            AutonomousTasks.parallelDriveAndArm(0.7,26,270,0.7, Constants.ARM_IN_TICKS_2 - 200, telemetry);

            timer.reset();
            while (timer.milliseconds() < 3000) {
                driveTrain.driveTank(0, -0.7);
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
