package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Drive;

import java.util.function.DoubleSupplier;

public class FieldCentric extends CommandBase {
    private final Drive s_drive;
    private final DoubleSupplier strafe, forward, turn;

    public FieldCentric(Drive s_drive, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){
        this.s_drive = s_drive;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        addRequirements(s_drive);
    }

    @Override
    public void initialize() {
        s_drive.resetGyro();

    }

    @Override
    public void execute(){
        s_drive.fieldCentricDrive(-strafe.getAsDouble(), -forward.getAsDouble(), -turn.getAsDouble());
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
