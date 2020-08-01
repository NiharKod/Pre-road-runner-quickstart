package org.firstinspires.ftc.teamcode.Main_Drive.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Main_Drive.RobotLibraries.LoggerEx;



public class Robot {
    HardwareMap hw;
    Telemetry telemetry;
    public  SimpleDrivetrain drivetrain;
    public  SimpleDrivetrain2 drivetrain2;
    public LoggerEx loggerEx;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.hw = hardwareMap;
        this.telemetry = telemetry;

    }
    public Robot(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry){
        this.hw = hardwareMap;
        this.telemetry = telemetry;

    }
    public void init(){
        drivetrain = new SimpleDrivetrain(hw);
        drivetrain2 = new SimpleDrivetrain2(hw);
        loggerEx = new LoggerEx(telemetry);
    }






}
