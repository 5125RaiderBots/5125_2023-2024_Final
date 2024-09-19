package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends SubsystemBase {
    Motor liftMotor1, liftMotor2;
    MotorGroup liftGroup;
    double TICS_TO_INCHES = ((1.0/360)*Math.PI*(32.25/25.4));
    Telemetry telemetry;
    int area;
    double target;
    double INS_TO_TICS = (360*(1.0/Math.PI)*(25.4/32.25));
    public enum Area{ZERO,BOTTOM, MIDDLE, HIGH}
    public Area e_area;

    double hold = 0.1;

    public Lift(HardwareMap hwMap, Telemetry telemetry){
        liftMotor1 = new Motor(hwMap, "lift1");
        liftMotor2 = new Motor(hwMap, "lift2");
        liftMotor2.setInverted(true);
        liftGroup = new MotorGroup(liftMotor1, liftMotor2);
        this.telemetry = telemetry;
        area=0;
        liftMotor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftMotor1.resetEncoder();
        liftMotor2.resetEncoder();
    }

    @Override
    public void periodic() {

        telemetry.addData("Area", e_area);
        telemetry.addData("liftTarget", (getTarget()*INS_TO_TICS)  );


    }

    public void run(double speed){
        liftGroup.set(speed+hold);
    }
    public void stop(){
        liftGroup.set(0);
    }
    public double getLiftPosition(){
         double one =(double)liftMotor1.getCurrentPosition()*TICS_TO_INCHES;
         double two =(double)liftMotor2.getCurrentPosition()*TICS_TO_INCHES;
         return (one+two)/2;

    }
    public void changeArea(int change){
        area = area + change;
        area = Math.max(0, Math.min(3, area));

        if (area == 0){
            e_area = Area.ZERO;
        }
        else if (area == 1){
            e_area = Area.BOTTOM;
        }
        else if(area==2){
            e_area = Area.MIDDLE;
        }
        else {
            e_area = Area.HIGH;
        }

    }

    public double getTarget(){

        switch (area){
            case 0: target = 0;
                break;
            case 1: target = 20;
                break;
            case 2: target = 25;
                break;
            case 3: target = 30;
                break;
        }
        return target;
    }

    public void setRunMode(Motor.RunMode runMode){
        liftGroup.setRunMode(runMode);
    }

    public boolean atTarget(){
        return liftGroup.atTargetPosition();
    }
    public void setTolerance(double tolerance){
        liftGroup.setPositionTolerance(tolerance);
    }

    public void target(){

        switch (area){
            case 0: target = 0;
                break;
            case 1: target = 30;
                break;
            case 2: target = 42;
                break;
            case 3: target = 52;
                break;
        }
        liftGroup.setTargetPosition((int)(target*INS_TO_TICS));
    }

    public void targetAuto(){
        liftGroup.setTargetPosition((int)(22*INS_TO_TICS));
    }



}
