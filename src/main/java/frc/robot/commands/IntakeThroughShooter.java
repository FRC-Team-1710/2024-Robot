// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeThroughShooter extends Command {
    private ShooterSubsystem shooter;
    private IntexerSubsystem intexer;
    private double stage = 0;
    private boolean finishPlease = false;

    /** Creates a new IntakeFromShooter. */
    public IntakeThroughShooter(ShooterSubsystem shooterSub, IntexerSubsystem intex) {
        shooter = shooterSub;
        intexer = intex;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSub, intex);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (stage == 0) {
            shooter.setShooterVelocity(-1000);
            intexer.setALL(-.5);
            shooter.setWristPosition(.66);
            if (intexer.intakeBreak()) {
                stage++;
            }
        } else if (stage == 1) {
            shooter.setShooterVelocity(Constants.Shooter.idleSpeedRPM);
            intexer.setALL(.5);
            shooter.setWristPosition(Constants.Shooter.intakeAngleRadians);
            if (intexer.shooterBreak()) {
                finishPlease = true;
            }
        }
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
        if (finishPlease) {
            return true;
        } else {
            return false;
        }
    }
}
