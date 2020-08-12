package org.firstinspires.ftc.teamcode.Main_Drive.OpMode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.SampleMecanumDrive;

@TeleOp(name = "IMUtest")
public class IMUtest extends OpMode {
    SampleMecanumDrive drive;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("IMU :", Math.toDegrees(angleWrap(drive.getRawExternalHeading())));
        telemetry.update();
    }

    public double angleWrap(double angle){
        return (angle + (2 * Math.PI)) % (2 * Math.PI);
    }

}

