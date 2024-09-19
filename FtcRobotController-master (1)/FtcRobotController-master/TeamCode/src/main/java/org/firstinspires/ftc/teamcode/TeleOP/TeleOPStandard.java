package org.firstinspires.ftc.teamcode.TeleOP;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.FieldCentric;
import org.firstinspires.ftc.teamcode.command.LiftDistance;
import org.firstinspires.ftc.teamcode.command.LiftManual;
import org.firstinspires.ftc.teamcode.command.RobotCentric;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Elbow;
import org.firstinspires.ftc.teamcode.subsystem.Mechs;
import org.firstinspires.ftc.teamcode.subsystem.Lift;


@TeleOp(name = "TeleOP")
public class TeleOPStandard extends CommandOpMode {
    Drive s_drive;
    Lift s_lift;
    Intake s_intake;
    Elbow s_elbow;
    Mechs s_launch;
    FieldCentric f_drive;
    RobotCentric r_drive;
    LiftManual m_lift;
    Command slow, fast, open, close, autoOpen, autoClose, reverse, standard;
    Command gateOpen, gateClose, autoGateOpen, autoGateClose;
    Command lifterUP,lifterDOWN, liftZoneUp, liftZoneDown, liftMax, liftMin;
    Command elbowScore, elbowReset, elbowTest;
    RunCommand intake, outtake;
    InstantCommand launch;
    ParallelCommandGroup score, reset;
    SequentialCommandGroup revert,goBack;
    GamepadEx gp1 ,gp2;
    InstantCommand scoreTest, resetTest;



    @Override
    public void initialize(){

       gp1 = new GamepadEx(gamepad1);
       gp2 = new GamepadEx(gamepad2);

       s_drive = new Drive(hardwareMap, telemetry);
       s_intake = new Intake(hardwareMap, telemetry);
       s_lift = new Lift(hardwareMap, telemetry);
       s_elbow = new Elbow(hardwareMap, telemetry);
       s_launch = new Mechs(hardwareMap, telemetry);

       m_lift = new LiftManual(s_lift,gp2::getLeftY);


        liftZoneUp = new InstantCommand(()-> s_lift.changeArea(1));
       liftZoneDown = new InstantCommand(()-> s_lift.changeArea(-1));
       liftMax = new InstantCommand(()-> s_lift.changeArea(3));
       liftMin = new InstantCommand(()-> s_lift.changeArea(-3));

       lifterUP = new LiftDistance(s_lift, false);
       lifterDOWN = new LiftDistance(s_lift, false);

     /*  intake = new RunCommand(s_intake::intake);
       outtake = new RunCommand(s_intake::outtake);*/

       elbowReset = new InstantCommand(s_elbow::reset);
       elbowScore = new InstantCommand(s_elbow::score);
       scoreTest = new InstantCommand(s_elbow::score);
       resetTest = new InstantCommand(s_elbow::reset);

       elbowTest = new InstantCommand(s_elbow::score);

       launch = new InstantCommand(s_launch::launch);

       f_drive = new FieldCentric(s_drive, gp1::getLeftX, gp1::getLeftY, gp1::getRightX);
       r_drive = new RobotCentric(s_drive, gp1::getLeftX, gp1::getLeftY, gp1::getRightX);


        slow = new InstantCommand(()->s_drive.setMult(.4));
        fast = new InstantCommand(()->s_drive.setMult(1));

        reverse = new ParallelCommandGroup( new InstantCommand(()->s_drive.setDirection(-1)), new InstantCommand(()->s_drive.setTurn(180)));
        standard = new ParallelCommandGroup( new InstantCommand(()->s_drive.setDirection(1)), new InstantCommand(()->s_drive.setTurn(0)));

        open = new InstantCommand(s_intake::open);
        autoOpen = new InstantCommand(s_intake::open);
        close = new InstantCommand(s_intake::close);
        autoClose = new InstantCommand(s_intake::close);

        gateOpen = new InstantCommand(s_intake::openGate);
        autoGateOpen = new InstantCommand(s_intake::openGate);
        gateClose = new InstantCommand(s_intake::closeGate);
        autoGateClose = new InstantCommand(s_intake::closeGate);

        //TODO: Scoring Command Groups

        revert = new SequentialCommandGroup(liftMin, lifterDOWN);

        score = new ParallelCommandGroup(
                lifterUP,
                new SequentialCommandGroup(
                        new WaitCommand(250), elbowScore)
        );

        goBack =  new SequentialCommandGroup(
                autoOpen, new WaitCommand(1000),
                new ParallelCommandGroup(
                        autoClose,
                        elbowReset,
                        new SequentialCommandGroup(
                                new WaitCommand(1000), revert))
        );


        gp1.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(r_drive,f_drive);
        gp1.getGamepadButton(GamepadKeys.Button.A).whenHeld(slow).whenReleased(fast);
        gp1.getGamepadButton(GamepadKeys.Button.B).toggleWhenPressed(reverse, standard);

        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ParallelCommandGroup(close, gateClose));
        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ParallelCommandGroup(open, gateOpen));

        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(liftZoneUp);
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(liftZoneDown);


        gp2.getGamepadButton(GamepadKeys.Button.Y).whileHeld(score).whenReleased(goBack);

        gp2.getGamepadButton(GamepadKeys.Button.A).whenPressed(launch);

        //gp2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(scoreTest);
        //gp2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(elbowTest);


       register(s_drive,s_lift);
       s_drive.setDefaultCommand(f_drive);
       s_lift.setDefaultCommand(m_lift);

    }

}

