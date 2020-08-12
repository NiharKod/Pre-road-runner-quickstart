package org.firstinspires.ftc.teamcode.GoToPointTest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.StandardTrackingWheelLocalizer;

@TeleOp(name = "GoToPoint")
public class GoToPoint extends OpMode {

    StandardTrackingWheelLocalizer localizer;
    MecanumBase drive;

    @Override
    public void init() {
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        drive = new MecanumBase(hardwareMap,telemetry);
        drive.init();


    }

    @Override
    public void loop() {
        drive.setPowerCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, localizer.getPoseEstimate().component3());

    }
}
