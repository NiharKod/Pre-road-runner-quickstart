package org.firstinspires.ftc.teamcode.Main_Drive.Libs;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LoggerEx {

    private Telemetry telemetry;

    public LoggerEx(Telemetry telemetry){
        this.telemetry = telemetry;

    }

    public void addData(String caption, Object value){
        telemetry.addData(caption, value);
        telemetry.update();
    }

    public void addLine(){

        telemetry.addLine();
        telemetry.update();
    }


}
