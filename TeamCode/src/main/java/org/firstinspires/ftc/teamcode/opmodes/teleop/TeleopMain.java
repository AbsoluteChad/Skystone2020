/*
Copyright 2018 FIRST Tech Challenge Team 12923

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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

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

@TeleOp(name="TeleopMain", group="Iterative Opmode")
public class TeleopMain extends OpMode {

    private RobotMain robot;

    private final double ELEVATOR_ARM_POWER = 0.25;
    private final double GRIPPER_SUCC_POWER = 0.5;

    @Override
    public void init() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        telemetry.addData("GO GO GO GO GO GO", "You pressed play early didn't you (@vid_dude)?");
        telemetry.update();
    }

    @Override
    public void loop() {
        /*for (Subsystem subsystem : RobotMain.allSubsystems) {
            subsystem.teleopTick();
        }*/

        //Drivetrain controls
        robot.driveMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);

        //Elevator arm controls
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            robot.driveElevatorArm(ELEVATOR_ARM_POWER);
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            robot.driveElevatorArm(-ELEVATOR_ARM_POWER);
        } else {
            robot.driveElevatorArm(0);
        }

        //Rotational arm controls
        robot.driveRotationalArm(gamepad2.left_stick_y);

        //Succ code
        double gripperClosePower;
        double gripperFarPower;
        if (gamepad1.y || gamepad2.y) {
            gripperClosePower = GRIPPER_SUCC_POWER;
            gripperFarPower = GRIPPER_SUCC_POWER;
        } else if (gamepad1.x || gamepad2.x) {
            gripperClosePower = -GRIPPER_SUCC_POWER;
            gripperFarPower = -GRIPPER_SUCC_POWER;
        } else {
            gripperClosePower = gamepad2.left_bumper ? -gamepad2.left_trigger : gamepad2.left_trigger;
            gripperFarPower = gamepad2.right_bumper ? -gamepad2.right_trigger : gamepad2.right_trigger;
        }
        robot.gripperSucc(gripperClosePower, gripperFarPower);
    }


    @Override
    public void stop() {
        robot.driveTank(0, 0);
        robot.driveElevatorArm(0);
        robot.driveRotationalArm(0);
        robot.gripperSucc(0, 0);
    }
}
