package org.firstinspires.ftc.teamcode.lib.drive;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotMain;

public class Pose2d {

    //Declare 3D positional objects
    private VectorF translation;
    private Orientation orientation;

    public Pose2d(VectorF translation, Orientation orientation) {
        this.translation = translation;
        this.orientation = orientation;
    }

    //Translation accessor methods
    public double getX() {
        return translation.get(0) / RobotMain.mmPerInch;
    }

    public double getY() {
        return translation.get(1) / RobotMain.mmPerInch;
    }

    public double getZ() {
        return translation.get(2) / RobotMain.mmPerInch;
    }

    //Orientation accessor methods
    public double getRoll() {
        return orientation.firstAngle;
    }

    public double getPitch() {
        return orientation.secondAngle;
    }

    public double getHeading() {
        return orientation.thirdAngle;
    }

    //Telemetry data
    @Override
    public String toString() {
        String s = "Position relative to field center (inches): " + "\n";
        s += "(X, Y, Z) = (" + getX() + ", " + getY() + ", " + getZ() + ") \n";
        s += "Orientation relative to field center (degrees): " + "\n";
        s += "(roll, pitch, heading) = (" + getRoll() + ", " + getPitch() + ", " + getHeading() + ")";
        return s;
    }
}
