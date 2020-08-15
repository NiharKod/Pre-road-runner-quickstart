package org.firstinspires.ftc.teamcode.Main_Drive.OpMode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.DriveConstants;
import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Main_Drive.Subsystems.Robot;


@Autonomous(name = "5StoneAuto")
public class FourStoneRed extends LinearOpMode {

    public SampleMecanumDrive drive;
    public Servo Claw;
    public Servo Arm;
    public double ARM_DOWN = 0.26;
    public double ARM_UP = .575;
    public double CLAW_OPEN = .385;
    public double CLAW_CLOSE = .9;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-62, -33, Math.toRadians(0)));
        Claw = hardwareMap.get(Servo.class, "Claw");
        Arm = hardwareMap.get(Servo.class, "Arm");


        waitForStart();
        if (isStopRequested()) return;

        Trajectory toStone1 = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()))
                .lineToConstantHeading(new Vector2d(-34, -60), DriveConstants.BASE_CONSTRAINTS)
                .addTemporalMarker(.5, this::setARM_DOWN)
                .addDisplacementMarker(this::setCLAW_CLOSE)
                .addDisplacementMarker(this::setARM_UP)
                .build();

        Trajectory toFoundation1 = drive.trajectoryBuilder(toStone1.end())
                .splineToConstantHeading(new Vector2d(-40,0),Math.toRadians(0))
                .addTemporalMarker(.5, this::setARM_DOWN)
                .splineToConstantHeading(new Vector2d(-36,52),Math.toRadians(0))
                .addDisplacementMarker(this::setCLAW_OPEN)
                .addDisplacementMarker(this::setARM_UP)
                .build();

        Trajectory toStone2 = drive.trajectoryBuilder(toFoundation1.end())
                .splineToConstantHeading(new Vector2d(-40,0),Math.toRadians(0))
                .addTemporalMarker(.5, this::setARM_DOWN)
                .splineToConstantHeading(new Vector2d(-34,-36),Math.toRadians(0))
                .addDisplacementMarker(this::setCLAW_CLOSE)
                .addDisplacementMarker(this::setARM_UP)
                .build();

        Trajectory toFoundation2 = drive.trajectoryBuilder(toStone2.end())
                .splineToConstantHeading(new Vector2d(-40,0),Math.toRadians(0))
                .addTemporalMarker(.5, this::setARM_DOWN)
                .splineToConstantHeading(new Vector2d(-34,52),Math.toRadians(0))
                .addDisplacementMarker(this::setCLAW_OPEN)
                .addDisplacementMarker(this::setARM_UP)
                .build();

        Trajectory toStone3= drive.trajectoryBuilder(toFoundation2.end())
                .splineToConstantHeading(new Vector2d(-40,0),Math.toRadians(0))
                .addTemporalMarker(.5, this::setARM_DOWN)
                .splineToConstantHeading(new Vector2d(-34,-20),Math.toRadians(0))
                .addDisplacementMarker(this::setCLAW_CLOSE)
                .addDisplacementMarker(this::setARM_UP)
                .build();

        Trajectory toFoundation3 = drive.trajectoryBuilder(toStone3.end())
                .splineToConstantHeading(new Vector2d(-40,0),Math.toRadians(0))
                .addTemporalMarker(.5, this::setARM_DOWN)
                .splineToConstantHeading(new Vector2d(-34,52),Math.toRadians(0))
                .addDisplacementMarker(this::setCLAW_OPEN)
                .addDisplacementMarker(this::setARM_UP)
                .build();

        Trajectory toStone4 = drive.trajectoryBuilder(toFoundation3.end())
                .splineToConstantHeading(new Vector2d(-40,0),Math.toRadians(0))
                .addTemporalMarker(.5, this::setARM_DOWN)
                .splineToConstantHeading(new Vector2d(-34,-28),Math.toRadians(0))
                .addDisplacementMarker(this::setCLAW_CLOSE)
                .addDisplacementMarker(this::setARM_UP)
                .build();

        Trajectory toFoundation4 = drive.trajectoryBuilder(toStone4.end())
                .splineToConstantHeading(new Vector2d(-40,0),Math.toRadians(0))
                .addTemporalMarker(.5, this::setARM_DOWN)
                .splineToConstantHeading(new Vector2d(-34,52),Math.toRadians(0))
                .addDisplacementMarker(this::setCLAW_OPEN)
                .addDisplacementMarker(this::setARM_UP)
                .build();

        Trajectory toPark = drive.trajectoryBuilder(toFoundation4.end())
                .lineToConstantHeading(new Vector2d(-40,0), DriveConstants.BASE_CONSTRAINTS)
                .build();

        drive.followTrajectory(toStone1);
        drive.followTrajectory(toFoundation1);
        drive.followTrajectory(toStone2);
        drive.followTrajectory(toFoundation2);
        drive.followTrajectory(toStone3);
        drive.followTrajectory(toFoundation3);
        drive.followTrajectory(toStone4);
        drive.followTrajectory(toFoundation4);
        drive.followTrajectory(toPark);
    }
    public void setARM_DOWN(){
        Arm.setPosition(ARM_DOWN);
    }
    public void setARM_UP(){
        Arm.setPosition(ARM_UP);
    }
    public void setCLAW_OPEN(){
        Claw.setPosition(CLAW_OPEN);
    }
    public void setCLAW_CLOSE(){
        Claw.setPosition(CLAW_CLOSE);
    }
}
