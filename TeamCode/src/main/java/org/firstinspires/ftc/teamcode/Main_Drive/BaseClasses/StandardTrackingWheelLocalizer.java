package org.firstinspires.ftc.teamcode.Main_Drive.BaseClasses;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = (60 / 25.4) / 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = (422 / 25.4); // in; distance between the left and right wheels //how much it turns
    public static double FORWARD_OFFSET = -1 * (153 / 25.4); // in; offset of the lateral wheel //

    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private ExpansionHubEx hub;

    private int LEFT_TELEMETRY_PORT = 3;
    private int RIGHT_TELEMETRY_PORT = 0;
    private int FRONT_TELEMETRY_PORT = 1;


    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));


        hub = hardwareMap.get(ExpansionHubEx.class, "OdoHub");

    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData revBulkData = hub.getBulkInputData();
        return Arrays.asList(
                encoderTicksToInches(-revBulkData.getMotorCurrentPosition(LEFT_TELEMETRY_PORT)),
                encoderTicksToInches(revBulkData.getMotorCurrentPosition(RIGHT_TELEMETRY_PORT)),
                encoderTicksToInches(-revBulkData.getMotorCurrentPosition(FRONT_TELEMETRY_PORT))
        );
    }
}
