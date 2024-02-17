// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevationManual extends Command {
    private ElevatorSubsystem m_elevatorSubsystem;
    double m_speed;
    DoubleSupplier axis;

    public ElevationManual(ElevatorSubsystem elevate, DoubleSupplier control) {
        m_elevatorSubsystem = elevate;
        axis = control;
        addRequirements(elevate);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double value = axis.getAsDouble();

        value = Math.pow(value, 3);
        
        m_elevatorSubsystem.ManSpin(value);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
