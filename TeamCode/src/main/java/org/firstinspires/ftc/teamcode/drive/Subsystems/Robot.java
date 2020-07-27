package org.firstinspires.ftc.teamcode.drive.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    HardwareMap hw;
   public  SimpleDrivetrain drivetrain;
    public Robot(HardwareMap hardwareMap){
        this.hw = hardwareMap;

    }
    public void init(){
        drivetrain = new SimpleDrivetrain(hw);
    }




}
