package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Lift;

import java.util.function.DoubleSupplier;

public class LiftManual extends CommandBase {
    Lift s_lift;
    DoubleSupplier power;

    public LiftManual(Lift s_lift, DoubleSupplier power){
        this.s_lift = s_lift;
        this.power = power;
        addRequirements(s_lift);
    }

    @Override
    public void execute() {
        s_lift.run(power.getAsDouble());
    }

}
