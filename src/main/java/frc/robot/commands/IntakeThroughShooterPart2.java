// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeThroughShooterPart2 extends Command {
    private ShooterSubsystem shooter;
    private IntexerSubsystem intexer;
    private boolean finishPlease = false;

    /** Creates a new IntakeFromShooterPart2. */
    public IntakeThroughShooterPart2(ShooterSubsystem shooterSub, IntexerSubsystem intex) {
        shooter = shooterSub;
        intexer = intex;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSub, intex);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!intexer.intakeBreak()) {
            finishPlease = true;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setShooterVelocity(Constants.Shooter.idleSpeedRPM);
        intexer.setALL(.5);
        shooter.setWristPosition(Constants.Shooter.intakeAngleRadians);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelocity(Constants.Shooter.idleSpeedRPM);
        intexer.setALL(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (finishPlease || intexer.shooterBreak()) {
            return true;
        } else {
            return false;
        }
    }
}
