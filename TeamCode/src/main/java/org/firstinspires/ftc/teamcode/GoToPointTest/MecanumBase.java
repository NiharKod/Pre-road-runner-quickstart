package org.firstinspires.ftc.teamcode.GoToPointTest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Vector3;

import java.util.Arrays;

public class MecanumBase {

    public HardwareMap hw;
    public PID_Controller _x_controller;
    public PID_Controller _y_controller;
    public PID_Controller _turn_controller;
    public Telemetry telemetry;
    public DcMotorEx FL;
    public DcMotorEx FR;
    public DcMotorEx BL;
    public DcMotorEx BR;
    public DcMotor leftEncoder;
    public DcMotor rightEncoder;
    public DcMotor strafeEncoder;

    public double kp = 0.03;
    public double ki = 0;
    public double kd = .0025;
    public double kpr = 0.6;
    public double kir = 0;
    public double kdr = 0.02;

    public MecanumBase(HardwareMap hw, Telemetry telemetry){
        this.hw = hw;
        this.telemetry = telemetry;

    }

    public void init(){
            FL = hw.get(DcMotorEx.class, "FL");
            FR = hw.get(DcMotorEx.class, "FR");
            BL = hw.get(DcMotorEx.class, "BL");
            BR = hw.get(DcMotorEx.class, "FR");

            FL.setDirection(DcMotorSimple.Direction.REVERSE);
            BL.setDirection(DcMotorSimple.Direction.REVERSE);


            leftEncoder = hw.get(DcMotor.class, "leftEncoder");
            rightEncoder = hw.get(DcMotor.class, "rightEncoder");
            strafeEncoder = hw.get(DcMotor.class, "frontEncoderl");

    }

    public void setPower(double x, double y, double rot){
        double FrontLeftVal = y - x + rot;
        double FrontRightVal = y + x - rot;
        double BackLeftVal = y + x + rot;
        double BackRightVal = y - x - rot;

        double[] power = {FrontLeftVal, FrontRightVal, BackLeftVal, BackRightVal};
        Arrays.sort(power);

        if(power[3] > 1 ) {
            FrontLeftVal /= power[3];
            FrontRightVal /= power[3];
            BackLeftVal /= power[3];
            BackRightVal /= power[3];
        }

        FL.setPower(FrontLeftVal);
        FR.setPower(FrontRightVal);
        BL.setPower(BackLeftVal);
        BR.setPower(BackLeftVal);
    }

    public void setPowerCentric(double x, double y, double rot, double heading){
        Vector3 power = new Vector3(x,y);
        power.rotate(heading);
        x = power.x;
        y = power.y;

        setPower(x,y,rot);

    }
    public void goToPoint(Pose2d target, Pose2d current){
        double heading = current.getHeading();
        double target_heading = target.getHeading();
        if(current.getHeading() >= Math.PI){
            heading = -((2 * Math.PI) - current.getHeading());
        }
        if(Math.abs(target.getHeading() - heading) >= Math.PI){
            target_heading = -((2 * Math.PI) - target.getHeading());
        }
        double _x_power = _x_controller.update(target.getX(), current.getX());
        double _y_power = _y_controller.update(target.getX(), current.getX());
        double _rot_power = _turn_controller.update(target_heading, heading);
        telemetry.addData("Y Power", String.valueOf(_y_power));

        setPowerCentric(-_x_power, _y_power, -_rot_power, current.getHeading());

    }
    public double getLeftOdo(){
        double currentTicks = leftEncoder.getCurrentPosition();
        double inches = (currentTicks * (60 * Math.PI) / 25.4) / 8192;

        return inches;

    }

    public double getRightOdo(){
        double currentTicks = rightEncoder.getCurrentPosition();
        double inches = (currentTicks * (60 * Math.PI) / 25.4) / 8192;

        return inches;
    }

    public double getStrafeOdo(){
        double currentTicks = strafeEncoder.getCurrentPosition();
        double inches = (currentTicks * (60 * Math.PI) / 25.4) / 8192;

        return inches;
    }


    public void setPowerZero(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

    }






}
