package org.firstinspires.ftc.teamcode.Main_Drive.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Drive.Libs.StickyGamepad;

public class Robot {
    HardwareMap hw;
   public  SimpleDrivetrain drivetrain;



    public Robot(HardwareMap hardwareMap){
        this.hw = hardwareMap;

    }
    public Robot(HardwareMap hardwareMap, Gamepad gamepad){
        this.hw = hardwareMap;

    }
    public void init(){
        drivetrain = new SimpleDrivetrain(hw);
    }




}
