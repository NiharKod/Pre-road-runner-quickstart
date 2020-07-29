package org.firstinspires.ftc.teamcode.Main_Drive.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.Vector3;

import java.util.Arrays;

public class SimpleDrivetrain2 {
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;
    public HardwareMap hw = null;
    public SimpleDrivetrain2(HardwareMap hardwareMap){ this.hw = hardwareMap;}


    public void init(){
        FL = hw.get(DcMotorEx.class, "FL");
        FR = hw.get(DcMotorEx.class, "FR");
        BL = hw.get(DcMotorEx.class, "BL");
        BR = hw.get(DcMotorEx.class, "BR");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void setPower(double x, double y, double theta){

        //power calculations
        double FrontLeftVal = y - x + theta;
        double FrontRightVal = y + x - theta;
        double BackLeftVal = y + x + theta;
        double BackRightVal = y - x - theta;
        //sort array to normalize
        double[] power = {FrontLeftVal, FrontRightVal, BackLeftVal, BackRightVal};
        Arrays.sort(power);
        //normalize
        FrontLeftVal /= power[3];
        FrontRightVal /= power[3];
        BackLeftVal /= power[3];
        BackRightVal /= power[3];
        //set power
        FL.setPower(FrontLeftVal);
        FR.setPower(FrontRightVal);
        BL.setPower(BackLeftVal);
        BR.setPower(BackRightVal);
    }

    public void setPowerFieldCentric(double x, double y, double rot, double heading){

        Vector3 vector = new Vector3(x,y);
        vector.rotate(heading);
        x = vector.x;
        y = vector.y;


        //power calculations
        double FrontLeftVal = y - x + rot;
        double FrontRightVal = y + x - rot;
        double BackLeftVal = y + x + rot;
        double BackRightVal = y - x - rot;
        //sort array to normalize
        double[] power = {FrontLeftVal, FrontRightVal, BackLeftVal, BackRightVal};
        Arrays.sort(power);
        //normalize
        FrontLeftVal /= power[3];
        FrontRightVal /= power[3];
        BackLeftVal /= power[3];
        BackRightVal /= power[3];
        //set power
        FL.setPower(FrontLeftVal);
        FR.setPower(FrontRightVal);
        BL.setPower(BackLeftVal);
        BR.setPower(BackRightVal);
    }

}
