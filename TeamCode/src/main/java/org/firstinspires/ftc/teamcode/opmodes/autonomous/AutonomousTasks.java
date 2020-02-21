package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatingArm;

public class AutonomousTasks {

    //Declare subsystems
    private static DriveTrain driveTrain = (DriveTrain) DriveTrain.getInstance();
    private static ElevatingArm elevatingArm = (ElevatingArm) ElevatingArm.getInstance();

    /**
     * Used for driving the drive train & arm at the same time to reduce time while still retaining distance values (inches forward
     * for drive train, raw ticks for arm). Returns from method once both subsystems are done moving.
     *
     * @param drivePower requested power for drive train
     * @param driveInches requested distance (inches) for drive train to travel
     * @param degreeDirection direction requested to travel by robot
     *                        0, 90, 180, and 270 will travel distance;
     *                        any other angle will not do anything.
     * @param armPower requested power for arm
     * @param armTicks requested distance (encoder ticks) for arm to travel
     */
    public static void parallelDriveAndArm(double drivePower, int driveInches,
                                           double degreeDirection, double armPower, double armTicks, Telemetry telemetry) {
        /**
         * DriveTrain init
         */
        if (degreeDirection % 90 != 0) {
            return;
        }

        //Determine component values
        double thrust = Math.sin(Math.toRadians(degreeDirection)) * drivePower;
        double strafe = Math.cos(Math.toRadians(degreeDirection)) * drivePower;

        //Ready encoders & set target positions (TLBR = topLeft & bottomRight motors; BLTR = bottomLeft & topRight motors)
        int driveSetPoint = driveTrain.toTicks(driveInches);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int TLBRSetpt = thrust + strafe > 0 ? driveSetPoint : -driveSetPoint;
        int BLTRSetpt = thrust - strafe > 0 ? driveSetPoint : -driveSetPoint;

        driveTrain.topLeft.setTargetPosition(TLBRSetpt);
        driveTrain.bottomLeft.setTargetPosition(BLTRSetpt);
        driveTrain.topRight.setTargetPosition(BLTRSetpt);
        driveTrain.bottomRight.setTargetPosition(TLBRSetpt);

        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

        /**
         * Arm init
         */
        elevatingArm.rotationalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int armSetPoint = (int) armTicks;
        elevatingArm.rotationalArm.setTargetPosition(armSetPoint);
        elevatingArm.rotationalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /**
         * Drive both subsystems
         */
        //Set powers
        elevatingArm.rotationalArm.setPower(armPower);
        driveTrain.driveMecanum(thrust, strafe, 0, false);
        boolean rotateBusy = true;
        //Wait for all motors to complete
        while ((driveTrain.topLeft.isBusy() || driveTrain.bottomLeft.isBusy() || driveTrain.topRight.isBusy()
                || driveTrain.bottomRight.isBusy()) || rotateBusy) {
            //             elevatingArm.rotationalArm.isBusy()) {
            //         elevatingArm.killArm(armPower);
            //Check if any individual subsystem has finished while the other continues
            telemetry.addData("topLeft busy: " ,driveTrain.topLeft.isBusy() );
            telemetry.addData("bottomLeft busy: " ,driveTrain.bottomLeft.isBusy() );
            telemetry.addData("topRight busy: " ,driveTrain.topRight.isBusy() );
            telemetry.addData("bottomRight busy: " ,driveTrain.bottomRight.isBusy() );

            if (!elevatingArm.rotationalArm.isBusy()) {
                rotateBusy = false;
            }
            telemetry.addData("rotationalArm busy: " ,rotateBusy);
            telemetry.update();
            if (!rotateBusy) {
                elevatingArm.rotationalArm.setPower(0);
            }

/*            //made last minute

            if(!driveTrain.topLeft.isBusy() && !driveTrain.bottomLeft.isBusy() && !driveTrain.topRight.isBusy()
                    && !driveTrain.bottomRight.isBusy())
            {
                elevatingArm.rotationalArm.setPower(0);
                rotateBusy = false;
                elevatingArm.rotationalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
*/
         /*   if (!driveTrain.topLeft.isBusy()) {
                driveTrain.driveTank(0, 0);
            } else if (!elevatingArm.rotationalArm.isBusy()) {
                elevatingArm.rotationalArm.setPower(0);
            }

          */
        }
        driveTrain.driveTank(0, 0);
        elevatingArm.rotationalArm.setPower(0);

        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatingArm.rotationalArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * [UNFINISHED]
     *
     * Used for driving the drive train & arm at the same time to reduce time while still retaining distance values (inches forward
     * for drive train, raw ticks for arm). Returns from method once both subsystems are done moving.
     *
     * @param drivePower requested power for drive train
     * @param driveInches requested distance (inches) for drive train to travel
     * @param degreeDirection direction requested to travel by robot
     *                        0, 90, 180, and 270 will travel distance;
     *                        any other angle will not do anything.
     * @param armPower requested power for arm
     */
    public static void parallelDriveAndArmLS(double drivePower, int driveInches, double degreeDirection, double armPower) {
        /**
         * DriveTrain init
         */
        if (degreeDirection % 90 != 0) {
            return;
        }

        //Determine component values
        double thrust = Math.sin(Math.toRadians(degreeDirection)) * drivePower;
        double strafe = Math.cos(Math.toRadians(degreeDirection)) * drivePower;

        //Ready encoders & set target positions (TLBR = topLeft & bottomRight motors; BLTR = bottomLeft & topRight motors)
        int driveSetPoint = driveTrain.toTicks(driveInches);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int TLBRSetpt = thrust + strafe > 0 ? driveSetPoint : -driveSetPoint;
        int BLTRSetpt = thrust - strafe > 0 ? driveSetPoint : -driveSetPoint;

        driveTrain.topLeft.setTargetPosition(TLBRSetpt);
        driveTrain.bottomLeft.setTargetPosition(BLTRSetpt);
        driveTrain.topRight.setTargetPosition(BLTRSetpt);
        driveTrain.bottomRight.setTargetPosition(TLBRSetpt);

        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

        /**
         * Arm init
         */


        /**
         * Drive both subsystems
         */
        //Set powers
        elevatingArm.rotationalArm.setPower(armPower);
        driveTrain.driveMecanum(thrust, strafe, 0, false);
        boolean rotateBusy = true;
        //Wait for all motors to complete
        while ((driveTrain.topLeft.isBusy() || driveTrain.bottomLeft.isBusy() || driveTrain.topRight.isBusy()
                || driveTrain.bottomRight.isBusy()) || rotateBusy) {
            //             elevatingArm.rotationalArm.isBusy()) {
            //         elevatingArm.killArm(armPower);
            //Check if any individual subsystem has finished while the other continues

            if (!elevatingArm.rotationalArm.isBusy()) {
                elevatingArm.rotationalArm.setPower(0);
            }

/*            //made last minute

            if(!driveTrain.topLeft.isBusy() && !driveTrain.bottomLeft.isBusy() && !driveTrain.topRight.isBusy()
                    && !driveTrain.bottomRight.isBusy())
            {
                elevatingArm.rotationalArm.setPower(0);
                rotateBusy = false;
                elevatingArm.rotationalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
*/
         /*   if (!driveTrain.topLeft.isBusy()) {
                driveTrain.driveTank(0, 0);
            } else if (!elevatingArm.rotationalArm.isBusy()) {
                elevatingArm.rotationalArm.setPower(0);
            }

          */
        }
        driveTrain.driveTank(0, 0);
        elevatingArm.rotationalArm.setPower(0);

        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatingArm.rotationalArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
