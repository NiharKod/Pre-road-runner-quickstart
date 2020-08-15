package org.firstinspires.ftc.teamcode.Main_Drive.OpMode.TestOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.internal.hardware.usb.ArmableUsbDevice;
import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.DriveConstants;
import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Main_Drive.Subsystems.Robot;


@Autonomous(group = "drive")
public class LineTest extends LinearOpMode {

    public SampleMecanumDrive drive;
    public Robot robot;
    public Servo Claw;
    public Servo Arm;

    @Override
    public void runOpMode() throws InterruptedException {

         drive = new SampleMecanumDrive(hardwareMap);
         robot = new Robot(hardwareMap, telemetry);
         drive.setPoseEstimate(new Pose2d(-62, -33, Math.toRadians(0)));
        Claw = hardwareMap.get(Servo.class, "Claw");
        Arm = hardwareMap.get(Servo.class, "Arm");

      //  grabStone();

        waitForStart();


        if (isStopRequested()) return;
        Trajectory toStone1 = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()))
                .lineToLinearHeading(new Pose2d(-34,-60, -Math.toRadians(90)), DriveConstants.BASE_CONSTRAINTS)
               // .addDisplacementMarker(() -> { grabStone();})
                .build();
        //drive.setPoseEstimate(new Pose2d(-34, -60, Math.toRadians(90)));

        Trajectory toBridge1 = drive.trajectoryBuilder(toStone1.end())
                .lineToConstantHeading(new Vector2d(-40,0), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory toAtFoundation1 = drive.trajectoryBuilder(toBridge1.end())
                .lineToConstantHeading(new Vector2d(-36,52), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory toMid1 = drive.trajectoryBuilder(toAtFoundation1.end())
                .lineToConstantHeading(new Vector2d(-40,0), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory toStone2 = drive.trajectoryBuilder(toMid1.end())
                .lineToConstantHeading(new Vector2d(-34,-36), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory toBridge2 = drive.trajectoryBuilder(toStone2.end())
                .lineToConstantHeading(new Vector2d(-40,0), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory toAtFoundation2 = drive.trajectoryBuilder(toBridge2.end())
                .lineToConstantHeading(new Vector2d(-36,52), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory toMid2 = drive.trajectoryBuilder(toAtFoundation2.end())
                .lineToConstantHeading(new Vector2d(-40,0), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory toStone3 = drive.trajectoryBuilder(toMid2.end())
                .lineToConstantHeading(new Vector2d(-34,-20), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory toBridge3 = drive.trajectoryBuilder(toStone3.end())
                .lineToConstantHeading(new Vector2d(-40,0), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory toAtFoundation3 = drive.trajectoryBuilder(toBridge3.end())
                .lineToConstantHeading(new Vector2d(-36,52), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory toMid3 = drive.trajectoryBuilder(toAtFoundation3.end())
                .lineToConstantHeading(new Vector2d(-40,0), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory toStone4 = drive.trajectoryBuilder(toMid3.end())
                .lineToConstantHeading(new Vector2d(-34,-28), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory toBridge4 = drive.trajectoryBuilder(toStone4.end())
                .lineToConstantHeading(new Vector2d(-40,0), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory toAtFoundation4 = drive.trajectoryBuilder(toBridge4.end())
                .lineToConstantHeading(new Vector2d(-36,52), DriveConstants.BASE_CONSTRAINTS)
                .build();

        Trajectory park = drive.trajectoryBuilder(toAtFoundation4.end())
                .lineToConstantHeading(new Vector2d(-40,0), DriveConstants.BASE_CONSTRAINTS)
                .build();











        drive.followTrajectory(toStone1);


        drive.followTrajectory(toBridge1);

        drive.followTrajectory(toAtFoundation1);
        drive.followTrajectory(toMid1);
        drive.followTrajectory(toStone2);
        drive.followTrajectory(toBridge2);
        drive.followTrajectory(toAtFoundation2);
        drive.followTrajectory(toMid2);
        drive.followTrajectory(toStone3);
        drive.followTrajectory(toBridge3);
        drive.followTrajectory(toAtFoundation3);
        drive.followTrajectory(toMid3);
        drive.followTrajectory(toStone4);
        drive.followTrajectory(toBridge4);
        drive.followTrajectory(toAtFoundation4);
        drive.followTrajectory(park);











    }


    }

