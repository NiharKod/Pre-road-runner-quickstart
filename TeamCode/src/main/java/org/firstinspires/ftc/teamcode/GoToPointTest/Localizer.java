package org.firstinspires.ftc.teamcode.GoToPointTest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Vector3;

public class Localizer {
    MecanumBase base;
    HardwareMap hw;
    Telemetry telemetry;
    Pose2d myPose = new Pose2d(0,0,0);
    double TRACK_WIDTH = 0;
    double STRAFE_WIDTH = 0;
    static final double TICKS_PER_REV = 8192; // in ticks
    static final double WHEEL_DIAMETER_INCHES = 2.3622; //in mm
    static final double TICKS_PER_INCH = TICKS_PER_REV / (2 * Math.PI * (WHEEL_DIAMETER_INCHES / 2));

    double prevstrafe = 0;
    double prevvert = 0;
    double prstrafe = 0;
    double prevheading;


    public Localizer(HardwareMap hw, Telemetry telemetry){
        base = new MecanumBase(hw, telemetry);
    }

    public Pose2d getPose(){return myPose;}

    public void updateReadings(){


        double vert = (base.getLeftOdo() + base.getRightOdo()) / 2;
        double dvert = vert - prevvert;
        double dtheta = getHeading() - prevheading;
        double dstrafe = (base.getStrafeOdo() - prevstrafe) - (STRAFE_WIDTH * dtheta);


        prevstrafe = base.getStrafeOdo();
        prevvert = vert;

        Vector3 offset = constantVeloTrack(dstrafe, prevheading, dvert, dtheta);
        myPose = new Pose2d(myPose.getX() + offset.x, myPose.getY() + offset.y, getHeading());

        prevheading = getHeading();

    }

    public Vector3 constantVeloTrack(double dstrafe, double prevheading, double dvert, double dtheta){
        double sinterm = 0;
        double costerm = 0;

        if(dtheta == 0){
            sinterm = 1.0 - dtheta * dtheta / 6.0;
            costerm = dtheta / 2.0;
        }else{
            sinterm = Math.sin(dtheta)/dtheta;
            costerm = (1 - Math.cos(dtheta))/dtheta;
        }

        Vector3 feildCentricOffset = new Vector3((dstrafe * sinterm) + (dvert * -costerm), (dstrafe * costerm) + (dvert * sinterm)).rotated(prevheading);
        return feildCentricOffset;
    }

    public double getHeading(){
        return angleWrap((base.getRightOdo() - base.getLeftOdo())/TRACK_WIDTH);
    }

    public double angleWrap(double angle){
        return (angle + (2 * Math.PI)) % (2 * Math.PI);
    }



}
