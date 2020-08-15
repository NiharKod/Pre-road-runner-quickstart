package org.firstinspires.ftc.teamcode.Main_Drive.OpMode.TestOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Main_Drive.Subsystems.Robot;

@Config
@TeleOp(group = "drive")
public class AutoClawTuner extends OpMode {
        public static double ArmDown = 1;
        public static double ArmUp = .2;
        public static double Armmiddle;
        public static double clawOpen = .3;
        public static double clawClose = 1;
        boolean open;
        boolean up;
        public Robot robot;
        public Servo Arm;
        public Servo Claw;
    @Override
    public void init() {
        //robot = new Robot(hardwareMap, telemetry);
        //robot.init();

        Arm = hardwareMap.get(Servo.class, "Arm");
        Claw = hardwareMap.get(Servo.class, "Claw");

        open  = false;
        up = false;

    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            Arm.setPosition(ArmDown);
        }
        if(gamepad1.b){
            Arm.setPosition(ArmUp);
        }

        if(gamepad1.x){
           Claw.setPosition(clawOpen);
        }
        if(gamepad1.y){
           Claw.setPosition(clawClose);
        }


    }
}
