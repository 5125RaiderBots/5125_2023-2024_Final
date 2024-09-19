package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.subsystem.Lift;

public class LiftDistance extends CommandBase {
    Lift s_lift;
    boolean isAuto;

    public LiftDistance(Lift s_lift, boolean isAuto){
       this.s_lift = s_lift;
       this.isAuto = isAuto;
    }

    @Override
    public void initialize() {
        s_lift.setTolerance(50);
        s_lift.setRunMode(Motor.RunMode.PositionControl);

        if (!isAuto){
        s_lift.target();}
        else{ s_lift.targetAuto();}
    }

    @Override
    public void execute() {
      s_lift.run(1);
    }

    @Override
    public void end(boolean interrupted) {
        s_lift.stop();
        s_lift.setRunMode(Motor.RunMode.RawPower);
    }

    @Override
    public boolean isFinished() {
       return s_lift.atTarget();
    }
}
