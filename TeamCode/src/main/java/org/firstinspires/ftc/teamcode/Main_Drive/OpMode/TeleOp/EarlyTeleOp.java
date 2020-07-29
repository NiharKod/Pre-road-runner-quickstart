package org.firstinspires.ftc.teamcode.Main_Drive.OpMode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Main_Drive.Subsystems.Robot;

public class EarlyTeleOp extends OpMode {

    Robot robot;
    SampleMecanumDrive drive;
    double heading = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.init();
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        // comment either setpower or setpower field centric
        drive.update();
        Pose2d poseEstimate = drive.getPoseEstimate();
        heading = poseEstimate.getHeading();

        //normal arcade
        robot.drivetrain.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        //field centric arcade
        robot.drivetrain.setPowerFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, heading);
    }
}