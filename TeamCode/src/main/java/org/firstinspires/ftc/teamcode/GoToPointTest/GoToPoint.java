package org.firstinspires.ftc.teamcode.GoToPointTest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GoToPoint")
public class GoToPoint extends OpMode {

    Localizer localizer;
    MecanumBase drive;

    @Override
    public void init() {
        localizer = new Localizer(hardwareMap, telemetry);
        drive = new MecanumBase(hardwareMap,telemetry);
        drive.init();


    }

    @Override
    public void loop() {
        drive.goToPoint(new Pose2d(-40,25,Math.toRadians(90)), localizer.getPose());
        localizer.updateReadings();
    }
}
