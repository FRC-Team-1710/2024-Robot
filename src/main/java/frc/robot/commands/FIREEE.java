// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.FiringSolutions;
import frc.robot.Constants;
import frc.robot.subsystems.IntexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FIREEE extends Command {
    private ShooterSubsystem shooter;
    private IntexerSubsystem intexer;

    public FIREEE(ShooterSubsystem shooterSub, IntexerSubsystem intex) {
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
        if (shooter.shooterAtSpeed()){
            intexer.setShooterIntake(.9);
        }
        shooter.PointShoot(shooter.getCalculatedAngle(), FiringSolutions.convertToRPM(shooter.getCalculatedVelocity()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intexer.setShooterIntake(0);
        shooter.SetShooterVelocity(Constants.Shooter.idleSpeedRPM);
        shooter.setWristPosition(Constants.Shooter.intakeAngleRadians);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
