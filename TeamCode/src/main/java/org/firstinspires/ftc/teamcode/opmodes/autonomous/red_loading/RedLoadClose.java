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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTasks;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatingArm;
import org.firstinspires.ftc.teamcode.subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import  org.firstinspires.ftc.teamcode.opmodes.Constants;

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
@Autonomous(name="RedLoadClose", group="Linear Opmode")
public class RedLoadClose extends LinearOpMode {

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

        int disToFoundation = 0; //this variable used for calculating distance to foundation

        waitForStart();

        if (opModeIsActive()) {
            //Go forward and sense
            //elevatingArm.rotationalArm.setPower(-0.2);
            driveTrain.driveDistance(1, 24, 90, false);
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

            elevatingArm.rotateArm(-0, Constants.ARM_OUT_TICKS, false);

            gripper.autoSucc(-1, 1500);
            telemetry.addData("checkpoint", 1);
            telemetry.update();

            elevatingArm.rotateArm(0.4, Constants.ARM_IN_TICKS, false);


            driveTrain.driveDistance(1, disToFoundation,0, false);
            AutonomousTasks.parallelDriveAndArm(1,14,90,-.7,
                    Constants.ARM_OUT_TICKS - 200, telemetry);


            //working stuff
            gripper.autoSucc(1, 1000);
            telemetry.addData("checkpoint", 1);
            telemetry.update();
            foundationMover.lockFoundation();


            AutonomousTasks.parallelDriveAndArm(1,19,270,.7,Constants.ARM_IN_TICKS,telemetry);

            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (timer.milliseconds() < 2500) {
                driveTrain.driveTank(0, -1);
            }
            driveTrain.driveDistance(0.7, 12, 90, false);
            foundationMover.unlockFoundation();
            driveTrain.driveDistance(1, 40, 270, false);

        }
    }
}
