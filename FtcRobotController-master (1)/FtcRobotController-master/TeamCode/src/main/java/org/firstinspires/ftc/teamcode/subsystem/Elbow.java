package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Elbow extends SubsystemBase {
    SimpleServo elbow1, elbow2;
    Telemetry telemetry;

    public Elbow(HardwareMap hwMap, Telemetry telemetry){
        elbow1 = new SimpleServo(hwMap, "elbow1",-0,360, AngleUnit.DEGREES);
        elbow2 = new SimpleServo(hwMap, "elbow2",-0,360, AngleUnit.DEGREES);


        elbow2.setInverted(true);
        elbow1.turnToAngle(0);
        elbow2.turnToAngle(0);
        this.telemetry=telemetry;
    }

    @Override
    public void periodic() {
        //telemetry.addData("Elbow1:", this.elbow1.getAngle());
    }

    //TODO: Elbow angle
    public void score(){
        elbow1.turnToAngle(90);
        elbow2.turnToAngle(90);
    }
    public void reset(){

        elbow1.turnToAngle(0);
        elbow2.turnToAngle(0);
    }
}
