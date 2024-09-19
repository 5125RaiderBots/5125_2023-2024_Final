package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
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

@Autonomous (name = "Red Close")
public class RedClose extends CommandOpMode {
    Drive s_drive;
    Lift s_lift;
    Intake s_claw;
    Elbow s_elbow;
    Mechs s_mechs;
    Command  x11;
    Command y11, y21, y31, y41,y51, y61;
    Command rx11, rx21, rx31;
    Command lifterUP1, lifterDOWN1, elbowScore1, elbowReset1, open11, open21, open31,close11, close21, close31;
    Command  x12,x22;
    Command y12, y22, y32, y42, y52;
    Command rx12, rx22, rx32;
    Command lifterUP2, lifterDOWN2, elbowScore2, elbowReset2, open12, open22, open32,close12, close22, close32;
    Command  x13,x23;
    Command y13, y23, y33, y43, y53;
    Command rx13, rx23, rx33;
    Command lifterUP3, lifterDOWN3, elbowScore3, elbowReset3, open13, open23, open33,close13, close23, close33;
    SelectCommand zone;
    ParallelCommandGroup auto1, auto2, auto3;
    Command drop1, drop2, drop3;
    Command clawHold1, clawHold2, clawHold3;

    @Override
    public void initialize() {
        s_drive = new Drive(hardwareMap, telemetry);
        s_lift = new Lift(hardwareMap,telemetry);
        s_claw = new Intake(hardwareMap,telemetry);
        s_elbow = new Elbow(hardwareMap, telemetry);
        s_mechs = new Mechs(hardwareMap, telemetry);

        x11 = new DriveDistanceX(s_drive, 4);
        y11 = new DriveDistanceY(s_drive,42);
        y21 = new DriveDistanceY(s_drive,-17);
        y31 = new DriveDistanceY(s_drive, -19);
        y41 = new DriveDistanceY(s_drive,5);
        y51 = new DriveDistanceY(s_drive,-22);
        y61 = new DriveDistanceY(s_drive, -20);
        rx11 = new TurnToAngle(s_drive,90);
        rx21 = new TurnToAngle(s_drive,0);
        lifterUP1 = new LiftDistance(s_lift, true);
        lifterDOWN1 = new LiftDistance(s_lift, false);
        elbowScore1 = new InstantCommand(s_elbow::score);
        open11 = new InstantCommand(s_claw::open);
        open21 = new InstantCommand(s_claw::open);
        open31 =   new InstantCommand(s_claw::open);
        close11 = new InstantCommand(s_claw::close);
        close21 = new InstantCommand(s_claw::close);
        close31 = new InstantCommand(s_claw::close);
        elbowReset1 = new InstantCommand(s_elbow::reset);
        drop1 = new InstantCommand(s_mechs::drop);



        x12 = new DriveDistanceX(s_drive, 6);
        x22 = new DriveDistanceX(s_drive,-10);
        y12 = new DriveDistanceY(s_drive,28);
        y22 = new DriveDistanceY(s_drive,-32);
        y32 = new DriveDistanceY(s_drive, -29);
        y42 = new DriveDistanceY(s_drive,5);
        y52 = new DriveDistanceY(s_drive,-20);
        rx12 = new TurnToAngle(s_drive,90);
        rx22 = new TurnToAngle(s_drive,-.01);
        lifterUP2 = new LiftDistance(s_lift, true);
        lifterDOWN2 = new LiftDistance(s_lift, false);
        elbowScore2 = new InstantCommand(s_elbow::score);
        open12 = new InstantCommand(s_claw::open);
        open22 = new InstantCommand(s_claw::open);
        open32 =   new InstantCommand(s_claw::open);
        close12 = new InstantCommand(s_claw::close);
        close22 = new InstantCommand(s_claw::close);
        close32 = new InstantCommand(s_claw::close);
        elbowReset2 = new InstantCommand(s_elbow::reset);
        drop2 = new InstantCommand(s_mechs::drop);

        x13 = new DriveDistanceX(s_drive, 6);
        x23 = new DriveDistanceX(s_drive, -1);
        y13 = new DriveDistanceY(s_drive,26);
        y23 = new DriveDistanceY(s_drive,18);
        y33 = new DriveDistanceY(s_drive, -52);
        y43 = new DriveDistanceY(s_drive,5);
        y53 = new DriveDistanceY(s_drive, -28);
        rx13 = new TurnToAngle(s_drive,90);
        rx23 = new TurnToAngle(s_drive,-.01);
        rx33 = new TurnToAngle(s_drive, 0);
        lifterUP3 = new LiftDistance(s_lift, true);
        lifterDOWN3 = new LiftDistance(s_lift, false);
        elbowScore3 = new InstantCommand(s_elbow::score);
        open13 = new InstantCommand(s_claw::open);
        open23 = new InstantCommand(s_claw::open);
        open33 =   new InstantCommand(s_claw::open);
        close13 = new InstantCommand(s_claw::close);
        close23 = new InstantCommand(s_claw::close);
        close33 = new InstantCommand(s_claw::close);
        elbowReset3 = new InstantCommand(s_elbow::reset);
        drop3 = new InstantCommand(s_mechs::drop);


        auto3 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                new InstantCommand(()-> s_drive.setMult(.5)),
                new ParallelCommandGroup(close11, y11),
                        new SequentialCommandGroup(lifterUP1, drop3, new WaitCommand(100), y21, rx11, y31, new WaitCommand(100), x11, new WaitCommand(100), y61, elbowScore1,
                new WaitCommand(500),
                open21,
                new WaitCommand(750),
                new ParallelCommandGroup(close31, y41)),
                new InstantCommand(()->s_lift.changeArea(-1)),
                new ParallelCommandGroup(elbowReset1, new SequentialCommandGroup(new WaitCommand(750), lifterDOWN1)),

                rx21, y51
        ), new RunCommand(()->s_lift.run(.04)));

        auto2 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                new InstantCommand(()-> s_drive.setMult(.5)),
                new ParallelCommandGroup(close12, x12), y12, rx12, lifterUP2, drop2,  y22,  x22,
                elbowScore2,
                new WaitCommand(500),
                open22,
                new WaitCommand(750),
                new ParallelCommandGroup(close32, y42),
                new InstantCommand(()->s_lift.changeArea(-1)),
                new ParallelCommandGroup(elbowReset2, new SequentialCommandGroup(new WaitCommand(750), lifterDOWN2)),
                rx22, y52
        ), new RunCommand(()->s_lift.run(.04)));

        auto1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                new InstantCommand(()-> s_drive.setMult(.5)),
                new ParallelCommandGroup(close13, y13), rx13, y23, lifterUP3,
                drop1, new WaitCommand(500), y33, x13, elbowScore3,
                new WaitCommand(500),
                open23,
                new WaitCommand(750),
                new ParallelCommandGroup(close33, y43),
                new InstantCommand(()->s_lift.changeArea(-1)),
                new ParallelCommandGroup(elbowReset3, new SequentialCommandGroup(new WaitCommand(1000), lifterDOWN3)),
                rx23,y53
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
