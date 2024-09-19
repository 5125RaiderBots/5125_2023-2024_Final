package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Drive;

public class TurnToAngle extends CommandBase {
    private final Drive s_drive;
    double angle;
    double tolerance = 1.1;

    public TurnToAngle(Drive s_drive, double angle){
        this.s_drive = s_drive;
        this.angle = angle;
    }

    @Override
    public void execute() {
        if (angle > 0){
        s_drive.robotCentricDrive(0,0,.5);}
        else {
            s_drive.robotCentricDrive(0,0,-.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_drive.stop();
    }

    @Override
    public boolean isFinished() {
         return Math.abs(s_drive.getYaw()- angle)<tolerance;
    }
}
