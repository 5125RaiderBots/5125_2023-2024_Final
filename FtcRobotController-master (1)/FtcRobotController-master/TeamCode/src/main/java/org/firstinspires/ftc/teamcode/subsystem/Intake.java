package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Intake extends SubsystemBase {

    SimpleServo lServo, rServo, gate;
    Telemetry telemetry;

    public Intake(HardwareMap hwMap, Telemetry telemetry){
        lServo = new SimpleServo(hwMap, "lServo", -180, 180, AngleUnit.DEGREES);
        rServo = new SimpleServo(hwMap, "rServo", -180, 180, AngleUnit.DEGREES);
        gate = new SimpleServo(hwMap, "gate", -180,180, AngleUnit.DEGREES);
        rServo.setInverted(true);
        this.telemetry = telemetry;
        lServo.turnToAngle(-15);
        rServo.turnToAngle(-15);
        gate.turnToAngle(0);

    }

    @Override
    public void periodic() {
       /* telemetry.addData("lServo", lServo.getAngle(AngleUnit.DEGREES));
        telemetry.addData("rServo", rServo.getAngle(AngleUnit.DEGREES));*/
    }

    //TODO: Claw angle
    public void open(){
        lServo.turnToAngle(15);
        rServo.turnToAngle(15);
    }
    public void close(){
        lServo.turnToAngle(-15);
        rServo.turnToAngle(-15);
    }

    public void openGate(){
        gate.turnToAngle(180);
    }

    public void closeGate(){
        gate.turnToAngle(0);
    }


}
