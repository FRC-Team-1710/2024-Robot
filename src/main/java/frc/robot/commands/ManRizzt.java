// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ManRizzt extends Command {

    ShooterSubsystem shooterSubsystem;
    private DoubleSupplier speed;

    public ManRizzt(ShooterSubsystem subsystem, DoubleSupplier speed) {
        // Use addRequirements() here to declare subsystem dependencies.
        shooterSubsystem = subsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speedValue = speed.getAsDouble() * 0.5;
        
        speedValue = Math.pow(speed.getAsDouble(), 3);

        shooterSubsystem.manualWristSpeed(speedValue);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.manualWristSpeed(0);
    }
}
