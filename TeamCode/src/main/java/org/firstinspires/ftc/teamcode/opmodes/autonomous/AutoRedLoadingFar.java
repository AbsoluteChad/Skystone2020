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
@Autonomous(name="LeAutonomousTest", group="Linear Opmode")

public class AutoRedLoadingFar extends LinearOpMode {

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

            //String skystonePosition = robot.tensorFlow.getSkystonePosition(true, 5000);
            String skystonePosition = "center";

            driveTrain.driveDistance(1, 21, 90, false);

            if (skystonePosition.equals("left")) {
                driveTrain.driveDistance(0.6, 18, 180, false);
                driveTrain.driveDistance(0.6, 12, 90, false);
                elevatingArm.rotateArm(-0.2, -2000, false);
                elevatingArm.rotateArm(0.2, -1600, false);
                driveTrain.driveDistance(0.6, 14, 270, false);
                driveTrain.driveDistance(0.7, 125,0, false); //107 or 90???
            } else if (skystonePosition.equals("center")) {
                elevatingArm.rotateArm(-0.05, -1800, false);
                //elevatingArm.rotateArm(-0.05, -500, false);
                //gri.pper.autoSucc(-1, 1500);
                elevatingArm.rotateArm(0.05, 100, false);
                //driveTrain.driveDistance(1, 107,0, false); //107 or 90???
                driveTrain.driveDistance(1, 2,0, false); //107 or 90???
                return;
            } else if (skystonePosition.equals("right")) {
                driveTrain.driveDistance(1, 18, 0, false);
                driveTrain.driveDistance(0.6, 12, 90, false);
                elevatingArm.rotateArm(-0.2, -1800, false);
                gripper.autoSucc(1, 1000);
                elevatingArm.rotateArm(0.2, -1600, false);
                driveTrain.driveDistance(0.6, 14, 270, false);
                driveTrain.driveDistance(0.7, 90,0, false); //107 or 90???
            } else if (skystonePosition.equals("nope")) {
                driveTrain.driveDistance(0.6, 12, 90, false);
                elevatingArm.rotateArm(-0.2, -1800, false);
                gripper.autoSucc(1, 1000);
                elevatingArm.rotateArm(0.2, -1600, false);
                driveTrain.driveDistance(0.6, 14, 270, false);
                driveTrain.driveDistance(0.7, 107,0, false); //107 or 90???
            } else {
                driveTrain.driveDistance(0.6, 12, 90, false);
                elevatingArm.rotateArm(-0.2, -1800, false);
                gripper.autoSucc(1, 1000);
                elevatingArm.rotateArm(0.2, -1600, false);
                driveTrain.driveDistance(0.6, 14, 270, false);
                driveTrain.driveDistance(0.7, 107,0, false); //107 or 90???
            }

            driveTrain.driveDistance(1, 12, 90, false);
            elevatingArm.rotateArm(-0.2, -1200, false);
            gripper.autoSucc(1, 2000);
            elevatingArm.rotateArm(-0.1, 600, false);
            foundationMover.lockFoundation();
            driveTrain.driveDistance(0.7, 30, 270, false);
            foundationMover.unlockFoundation();
            driveTrain.driveDistance(0.7, 6, 270, false);
            driveTrain.driveDistance(0.6, 15, 180, false);
            driveTrain.driveDistance(0.6, 32, 90, false);
            driveTrain.driveDistance(0.6, 29, 180, false);

            //elevatingArm.rotateArm(-0.2, -2000, false); //can go less than 2000

            /*//String skystonePos = robot.getSkystonePosition(false, 0);

            driveTrain.driveDistance(0.7, 12, 90, false);
            //lower arm to get skystone
            //move back a bit?? maybe.....
            driveTrain.driveDistance(0.7, 16, 180, false);
            driveTrain.rotateDegrees(0.7, -635);
            driveTrain.driveDistance(0.7, 107,90, false);
            driveTrain.rotateDegrees(0.7, 635);
            driveTrain.driveDistance(0.7, 16, 90, false);
            foundationMover.lockFoundation();
            driveTrain.driveDistance(0.7, 23, 270, false);
            foundationMover.unlockFoundation();
            while (opModeIsActive()){}

            /* //Red Alliance Code
            //Sensor code
            driveTrain.driveDistance(0.7, 11,90, false);
            //pick up block code
            driveTrain.driveDistance(0.5, 6,180, false);
            driveTrain.rotateDegrees(0.5, -635);
            driveTrain.driveDistance(0.7, 78,90, false);
            driveTrain.rotateDegrees(0.5, 635);
            foundationMover.lockFoundation();

            //Blue Alliance Code
            //Sensor
            if (skystonePos.equals("left")) {
                driveTrain.driveDistance(0.7, 11, 90, false);
                driveTrain.driveDistance(0.7, 6, 180, false);
                //pick up skystone code
                driveTrain.driveDistance(0.5, 6,180, false);
                //pickup skystone
                driveTrain.rotateDegrees(0.5, 635);
                driveTrain.driveDistance(0.5, 72,90, false);
                driveTrain.rotateDegrees(0.5, -635);
            }

            driveTrain.driveDistance(0.5, 6,180, false);
            driveTrain.rotateDegrees(0.5, 635);
            driveTrain.driveDistance(0.7, 78,90, false);
            driveTrain.rotateDegrees(0.5, -635);
            foundationMover.lockFoundation(); */
        }
    }
}
