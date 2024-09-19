package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Mechs extends SubsystemBase {
    Telemetry telemetry;
    SimpleServo launcher, dropper;
    public Mechs(HardwareMap hwMap, Telemetry telemetry){
        launcher = new SimpleServo(hwMap, "launch", -180, 180, AngleUnit.DEGREES);
        dropper = new SimpleServo(hwMap, "drop", -180, 180, AngleUnit.DEGREES);
        launcher.turnToAngle(0);
        dropper.turnToAngle(0);
        this.telemetry = telemetry;
        launcher.setInverted(true);
    }

    public void launch(){
        launcher.turnToAngle(90);
    }

    public void drop(){dropper.turnToAngle(180);}

}
