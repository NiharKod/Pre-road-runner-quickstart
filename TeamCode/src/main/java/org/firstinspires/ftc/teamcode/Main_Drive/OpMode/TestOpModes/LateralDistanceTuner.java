package org.firstinspires.ftc.teamcode.Main_Drive.OpMode.TestOpModes;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GoToPointTest.Localizer;
import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Main_Drive.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Main_Drive.Subsystems.SimpleDrivetrain;
//TODO: By turning the robot 180 degrees 11 times (odd number) this will calculate your lateral distance
@Autonomous(group = "Drive")
public class LateralDistanceTuner extends LinearOpMode {
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() {
        double left = 0;
        double right = 0;
        double lateralDistance = 0;
        drive = new SampleMecanumDrive(hardwareMap);
        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Wheel Positions",localizer.getWheelPositions());
            left = localizer.getWheelPositions().get(0);
            right = localizer.getWheelPositions().get(1);
            lateralDistance = (right + left) / 11 * Math.PI;
            telemetry.addData("Lateral Distance", lateralDistance);
            telemetry.update();
            /*
            for(int i = 0; i < 10; i++){
                drive.turn(Math.toRadians(180));
                sleep(300);
                telemetry.addData("Turns Occured", i + 1);
                telemetry.update();
            }
             */
        }
    }
}


