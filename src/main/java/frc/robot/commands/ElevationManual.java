// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevationManual extends Command {
    private ElevatorSubsystem mElevatorSubsystem;
    double m_speed;
    DoubleSupplier leftAxis;
    DoubleSupplier rightAxis;

    /** Creates a new VaderMan. */
    public ElevationManual(ElevatorSubsystem elevate, DoubleSupplier left, DoubleSupplier right) {
        mElevatorSubsystem = elevate;
        leftAxis = left;
        rightAxis = right;

        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = rightAxis.getAsDouble() - leftAxis.getAsDouble();
        mElevatorSubsystem.ManSpin(x);
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
