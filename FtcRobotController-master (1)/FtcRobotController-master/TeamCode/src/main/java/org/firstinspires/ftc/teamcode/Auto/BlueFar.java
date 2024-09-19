package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.command.DriveDistanceX;
import org.firstinspires.ftc.teamcode.command.DriveDistanceY;

import org.firstinspires.ftc.teamcode.command.LiftDistance;
import org.firstinspires.ftc.teamcode.command.TurnToAngle;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Elbow;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.Mechs;

import java.util.HashMap;

@Autonomous (name = "Blue Far")
public class BlueFar extends CommandOpMode {
    Drive s_drive;
    Lift s_lift;
    Intake s_claw;
    Elbow s_elbow;
    Mechs s_mechs;
    Command  x11, x21;
    Command y11, y21, y31, y41;
    Command rx11, rx21;
    Command lifterUP11, lifterUP21, lifterDOWN11,lifterDOWN21, elbowScore1, elbowReset1, open11, open21, open31,close11, close21, close31;
    Command  x12, x22, x32;
    Command y12, y22, y32, y42, y52;
    Command rx12, rx22;
    Command lifterUP12,lifterUP22, lifterDOWN12,lifterDOWN22, elbowScore2, elbowReset2, open12, open22, open32,close12, close22, close32;
    Command x13, x23;
    Command y13, y23, y33, y43,y53;
    Command rx13, rx23, rx33;
    Command lifterUP13, lifterUP23, lifterDOWN13, lifterDOWN23, elbowScore3, elbowReset3, open13, open23, open33,close13, close23, close33;
    SelectCommand zone;
    ParallelDeadlineGroup auto1, auto2, auto3;
    Command drop1, drop2, drop3;

    @Override
    public void initialize() {
        s_drive = new Drive(hardwareMap, telemetry);
        s_lift = new Lift(hardwareMap, telemetry);
        s_claw = new Intake(hardwareMap, telemetry);
        s_elbow = new Elbow(hardwareMap, telemetry);
        s_mechs = new Mechs(hardwareMap, telemetry);

        x11 = new DriveDistanceX(s_drive,4);
        x21 = new DriveDistanceX(s_drive, 22);
        y11 = new DriveDistanceY(s_drive,40);
        y21 = new DriveDistanceY(s_drive,10.5);
        y31 = new DriveDistanceY(s_drive, -79);
        y41 = new DriveDistanceY(s_drive,5);
        rx11 = new TurnToAngle(s_drive,-90);
        rx21 = new TurnToAngle(s_drive,0.01);
        lifterUP11 = new LiftDistance(s_lift, true);
        lifterUP21 = new LiftDistance(s_lift,true);
        lifterDOWN11 = new LiftDistance(s_lift, false);
        lifterDOWN21 = new LiftDistance(s_lift, false);
        elbowScore1 = new InstantCommand(s_elbow::score);
        open11 = new InstantCommand(s_claw::open);
        open21 = new InstantCommand(s_claw::open);
        open31 =   new InstantCommand(s_claw::open);
        close11 = new InstantCommand(s_claw::close);
        close21 = new InstantCommand(s_claw::close);
        close31 = new InstantCommand(s_claw::close);
        elbowReset1 = new InstantCommand(s_elbow::reset);
        drop1 = new InstantCommand(s_mechs::drop);

        x12 = new DriveDistanceX(s_drive, 13);
        x22 = new DriveDistanceX(s_drive, -7                                                                                                                                            );
        x32 = new DriveDistanceX(s_drive, 21);
        y22 = new DriveDistanceY(s_drive,32);
        y32 = new DriveDistanceY(s_drive, 20);
        y42 = new DriveDistanceY(s_drive,-82 );
        y52 = new DriveDistanceY(s_drive, 5);
        rx12 = new TurnToAngle(s_drive,-90);
        rx22 = new TurnToAngle(s_drive,0.001);
        lifterUP12 = new LiftDistance(s_lift, true);
        lifterUP22 = new LiftDistance(s_lift, true);
        lifterDOWN12 = new LiftDistance(s_lift, false);
        lifterDOWN22 = new LiftDistance(s_lift, false);
        elbowScore2 = new InstantCommand(s_elbow::score);
        open12 = new InstantCommand(s_claw::open);
        open22 = new InstantCommand(s_claw::open);
        open32 =   new InstantCommand(s_claw::open);
        close12 = new InstantCommand(s_claw::close);
        close22 = new InstantCommand(s_claw::close);
        close32 = new InstantCommand(s_claw::close);
        elbowReset2 = new InstantCommand(s_elbow::reset);
        drop2 = new InstantCommand(s_mechs::drop);

        x13 = new DriveDistanceX(s_drive, 25);
        x23 = new DriveDistanceX(s_drive, 30);
        y13 = new DriveDistanceY(s_drive,24);
        y23 = new DriveDistanceY(s_drive,16);
        y33 = new DriveDistanceY(s_drive, -16);
        y43 = new DriveDistanceY(s_drive,-76.5);
        y53 = new DriveDistanceY(s_drive, 5);
        rx13 = new TurnToAngle(s_drive,90);
        rx23 = new TurnToAngle(s_drive,-90);
        rx33 = new TurnToAngle(s_drive, .001);
        lifterUP13 = new LiftDistance(s_lift, true);
        lifterUP23 = new LiftDistance(s_lift,true);
        lifterDOWN13 = new LiftDistance(s_lift, false);
        lifterDOWN23 = new LiftDistance(s_lift, false);
        elbowScore3 = new InstantCommand(s_elbow::score);
        open13 = new InstantCommand(s_claw::open);
        open23 = new InstantCommand(s_claw::open);
        open33 =   new InstantCommand(s_claw::open);
        close13 = new InstantCommand(s_claw::close);
        close23 = new InstantCommand(s_claw::close);
        close33 = new InstantCommand(s_claw::close);
        elbowReset3 = new InstantCommand(s_elbow::reset);
        drop3 = new InstantCommand(s_mechs::drop);

        auto3 = new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                new InstantCommand(()-> s_drive.setMult(.5)),
                new ParallelCommandGroup(close11, y11),
                drop3, new WaitCommand(100), x11, lifterUP11, y21, lifterDOWN11, rx11,
                new InstantCommand(()->s_drive.setMult(.75)),
                new WaitCommand(8000),
                y31, x21,
                new ParallelCommandGroup(lifterUP21, new SequentialCommandGroup(new WaitCommand(750), elbowScore1)),
                new WaitCommand(500),
                open21,
                new WaitCommand(750),
                close31,
                new ParallelCommandGroup(elbowReset1, new SequentialCommandGroup(new WaitCommand(750), lifterDOWN21)),
                y41, rx21
        ),  new RunCommand(()->s_lift.run(.04)));

        auto2 = new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                new InstantCommand(()-> s_drive.setMult(.5)),
                new ParallelCommandGroup(close12, x12),
                y22, drop2, new WaitCommand(100), x22, lifterUP12, y32, lifterDOWN12, rx12,
                new InstantCommand(()->s_drive.setMult(.75)),
                new WaitCommand(8000),
                y42, x32,
                new ParallelCommandGroup(lifterUP22, new SequentialCommandGroup(new WaitCommand(750), elbowScore2)),
                new WaitCommand(500),
                open22,
                new WaitCommand(750),
                close32,
                new ParallelCommandGroup(elbowReset2, new SequentialCommandGroup(new WaitCommand(750), lifterDOWN22)),
                y52, rx22
        ), new RunCommand(()->s_lift.run(.04)));

        auto1 = new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                new InstantCommand(()-> s_drive.setMult(.5)),
                new ParallelCommandGroup(close13, y13), rx13,
                y23,
                new WaitCommand(200),
                drop1,
                new WaitCommand(200),
                lifterUP13,  y33, lifterDOWN13, x13,rx23,
                new InstantCommand(()->s_drive.setMult(.75)),
                new WaitCommand(3000),
                y43,x23,
                new ParallelCommandGroup(lifterUP23, new SequentialCommandGroup(new WaitCommand(750), elbowScore3)),
                new WaitCommand(500),
                open23,
                new WaitCommand(750),
                close33,
                new ParallelCommandGroup(elbowReset3, new SequentialCommandGroup(new WaitCommand(750), lifterDOWN23)),
                y53,rx33
        ), new RunCommand(()->s_lift.run(.04)));

        zone = new SelectCommand(
                new HashMap<Object, Command>() {{
                    put(Drive.Zone.LEFT, auto1);
                    put(Drive.Zone.RIGHT, auto2);
                    put(Drive.Zone.CENTER, auto3);
                }}, s_drive::checkZone
        );

        schedule(
                new SequentialCommandGroup(
                        new WaitCommand(250),
                        new InstantCommand(()-> s_drive.getZone()),
                        new InstantCommand(s_drive::camOff), zone));

    }

}
