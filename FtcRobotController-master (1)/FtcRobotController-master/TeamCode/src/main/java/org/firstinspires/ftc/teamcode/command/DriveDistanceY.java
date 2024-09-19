package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Drive;

public class DriveDistanceY extends CommandBase {
    private final Drive s_drive;
    private final double distance;
    double tolerance = 3;

    public DriveDistanceY(Drive s_drive, double distance){
        this.s_drive = s_drive;
        this.distance = distance;
        addRequirements(s_drive);
    }

    @Override
    public void initialize() {
        s_drive.resetEncoders();
    }

    @Override
    public void execute() {
        if (distance>0){
            s_drive.robotCentricDrive(0,-.75,0);}
        else {s_drive.robotCentricDrive(0,.75,0);}
    }

    @Override
    public void end(boolean interrupted) {
        s_drive.stop();
    }

    @Override
    public boolean isFinished() {
       return Math.abs(Math.abs((s_drive.getLEnc().getAsDouble()))-Math.abs(distance)) < tolerance;}
}
