package org.firstinspires.ftc.teamcode.Main_Drive.OpMode.TestOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.DriveConstants;
import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.DriveConstants.BASE_CONSTRAINTS;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();








        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);


    }
}
