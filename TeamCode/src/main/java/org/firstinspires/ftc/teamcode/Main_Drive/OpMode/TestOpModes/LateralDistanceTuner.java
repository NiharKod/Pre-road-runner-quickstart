package org.firstinspires.ftc.teamcode.Main_Drive.OpMode.TestOpModes;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.GoToPointTest.Localizer;
import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Main_Drive.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Main_Drive.Subsystems.SimpleDrivetrain;
//TODO: By turning the robot 180 degrees 11 times (odd number) this will calculate your lateral distance
@TeleOp(name = "LateralDistanceTuner")
public class LateralDistanceTuner extends LinearOpMode {
    SampleMecanumDrive drive;
    StandardTrackingWheelLocalizer localizer;
    double left = 0;
    double right = 0;
    double lateralDistance = 0;
    Orientation heading;
    float currentHeading;
    float myHeading = 0f;
    float targetHeading = (float)Math.toRadians(180);
    double varDouble;
    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        targetHeading = 0f;

        waitForStart();
        while(opModeIsActive()){

            updateTelemetryReadings();
/*
            for(int i = 0; i < 10; i++) {

                while (myHeading < targetHeading) {
                    myHeading = getOrientation();
                    targetHeading = myHeading + ((float)Math.toRadians(180));
                    drive.setMotorPowers(-.5, -.5, .5, .5);
                    updateTelemetryReadings();
                }
                drive.setMotorPowers(0, 0, 0, 0);
                sleep(500);
            }


 */
            updateTelemetryReadings();

        }

    }
    public void updateTelemetryReadings(){
        telemetry.addData("Wheel Positions",localizer.getWheelPositions());
        left = localizer.getWheelPositions().get(0);
        right = localizer.getWheelPositions().get(1);
        lateralDistance = (right + left) / 11 * Math.PI;
        telemetry.addData(" Approx : Lateral Distance", lateralDistance);
        telemetry.update();
    }
    public float getOrientation(){
        heading = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        drive.imu.getPosition();
        currentHeading = heading.firstAngle;

        return currentHeading;

    }

}


