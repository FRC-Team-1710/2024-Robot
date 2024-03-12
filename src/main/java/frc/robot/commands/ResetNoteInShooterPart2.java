// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ResetNoteInShooterPart2 extends Command {
    private ShooterSubsystem shooter;
    private IntexerSubsystem intexer;

    Joystick controller;
    public final Timer timer = new Timer();

    /** Creates a new IntakeFromShooterPart2. */
    public ResetNoteInShooterPart2(ShooterSubsystem shooterSub, IntexerSubsystem intex, Joystick controller) {
        shooter = shooterSub;
        intexer = intex;
        this.controller = controller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSub, intex);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        controller.setRumble(RumbleType.kBothRumble, 0);
        shooter.setWristByAngle(Constants.Shooter.intakeAngleRadians);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intexer.setALL(Constants.Intake.noteInsideSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intexer.setALL(0);
        intexer.setIntakeThroughShooterPart2Status(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (intexer.shooterBreak() || timer.get() > 1.5) {
            return true;
        } else {
            return false;
        }
    }
}
