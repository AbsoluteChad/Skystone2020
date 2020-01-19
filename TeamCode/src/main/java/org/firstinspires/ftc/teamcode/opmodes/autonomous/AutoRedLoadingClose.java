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
package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.*;

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
@Autonomous(name="AutoRedLoadingClose", group="Linear Opmode")

public class AutoRedLoadingClose extends LinearOpMode {

    private RobotMain robot;
    private DriveTrain driveTrain;
    private FoundationMover foundationMover;
    private ElevatingArm elevatingArm;
    private Gripper gripper;
    private Telemetry telemetry;

    private static int BLOCK_WIDTH = 10;
    private static int STRAFE_DIS_TO_FOUNDATION = 110;

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

            //String skystonePosition = robot.tensorFlow.getSkystonePosition(true, 5000);
            String skystonePosition = "center";


            driveTrain.driveDistance(1, 24, 90, false);

            if ((skystonePosition.equals("center")) || (skystonePosition.equals("nope"))){
                disToFoundation = STRAFE_DIS_TO_FOUNDATION;
            } else if (skystonePosition.equals("left")){
                disToFoundation = STRAFE_DIS_TO_FOUNDATION + BLOCK_WIDTH ;
                driveTrain.driveDistance(1, BLOCK_WIDTH, 180, false);
            } else if (skystonePosition.equals("right")) {
                disToFoundation = STRAFE_DIS_TO_FOUNDATION - BLOCK_WIDTH ;
                driveTrain.driveDistance(1, BLOCK_WIDTH, 0, false);
            }

            elevatingArm.rotateArm(0.7, -3400, false);
            gripper.autoSucc(-1, 1500);
            elevatingArm.rotateArm(0.7, 2340, false);

            driveTrain.driveDistance(1, disToFoundation,0, false);
            driveTrain.driveDistance(1, 12, 90, false);
            elevatingArm.rotateArm(0.7, -2340, false);

            gripper.autoSucc(1, 1000);
            foundationMover.lockFoundation();
            driveTrain.driveDistance(1, 35, 270, false);
            //driveTrain.rotateDegrees(0.7,-330);
            foundationMover.unlockFoundation();
            driveTrain.driveDistance(1, 23, 180, false);
            //driveTrain.driveDistance(1, 5, 90, false);
            //driveTrain.driveMecanum(0.7, 135, 3000);
            driveTrain.driveDistance(1, 30, 180, false);
        }
    }
}
