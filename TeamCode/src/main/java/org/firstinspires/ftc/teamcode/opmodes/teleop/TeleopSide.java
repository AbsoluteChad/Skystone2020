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
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousTasks;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatingArm;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp(name="TeleopSide", group="Iterative Opmode")
@Disabled
public class TeleopSide extends OpMode {

    private RobotMain robot;
    private ElevatingArm elevatingArm;
    private DriveTrain driveTrain;


    @Override
    public void init() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "blue", true);
        driveTrain = (DriveTrain) RobotMain.driveTrain;
        elevatingArm = (ElevatingArm) RobotMain.elevatingArm;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        /*if (gamepad1.dpad_down) {
            elevatingArm.rotateArmTest(-0.1, telemetry);
        } else if (gamepad1.dpad_up) {
            elevatingArm.rotateArmTest(0.1, telemetry);
        } else {
            elevatingArm.driveElevatorArm(0);
            driveTrain.driveTank(0,0);
        }

        if (gamepad1.y) {
            elevatingArm.rotateArm(0.1, 600, false, telemetry);
        }
        if (gamepad1.dpad_down) {
            AutonomousTasks.parallelDriveAndArm(0.7,14,90,.7,-2400,telemetry);
        } else if (gamepad1.dpad_up) {
            AutonomousTasks.parallelDriveAndArm(0.7,14,90,.7,-2400,telemetry);
        } else {
            elevatingArm.driveElevatorArm(0);
            driveTrain.driveTank(0,0);
          } */
    }



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("is pressed?", elevatingArm.inSensor.getState());
        if (elevatingArm.inSensor.getState() == true) {
            telemetry.addData("is pressed?", "not pressed");
        } else {
            telemetry.addData("is pressed?", "pressed");
        }
        telemetry.update();


        /*if (gamepad1.dpad_down) {
            AutonomousTasks.parallelDriveAndArm(0.7,14,90,.7,-2400,telemetry);
        } else if (gamepad1.dpad_up) {
            AutonomousTasks.parallelDriveAndArm(0.7,14,90,.7,-2400,telemetry);
        } else {
            elevatingArm.driveElevatorArm(0);
            driveTrain.driveTank(0,0);

        }

        if (gamepad1.y) {
            elevatingArm.rotateArm(0.5, -1440, false, telemetry);
            elevatingArm.driveElevatorArm(0);
        } else if (gamepad1.a) {
            elevatingArm.rotateArm(0.5, -3000, false, telemetry);
            elevatingArm.driveElevatorArm(0);
        }else {
            elevatingArm.driveElevatorArm(0);
        } */
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //test change
    }
}
