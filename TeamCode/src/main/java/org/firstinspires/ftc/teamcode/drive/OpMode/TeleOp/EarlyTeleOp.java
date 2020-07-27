package org.firstinspires.ftc.teamcode.drive.OpMode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.BaseClasses.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Subsystems.SimpleDrivetrain;

public class EarlyTeleOp extends OpMode {

    SimpleDrivetrain drivetrain;
    SampleMecanumDrive drive;
    double heading = 0;

    @Override
    public void init() {
        drivetrain = new SimpleDrivetrain(hardwareMap);
        drivetrain.init();
        drive = new SampleMecanumDrive(hardwareMap);

    }

    @Override
    public void loop() {
        // comment either setpower or setpower field centric
        drive.update();
        Pose2d poseEstimate = drive.getPoseEstimate();
        heading = poseEstimate.getHeading();

        //normal arcade
        drivetrain.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        //field centric arcade
        drivetrain.setPowerFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, heading);


    }
}
