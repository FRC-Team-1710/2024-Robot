// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSet extends Command {
    private ElevatorSubsystem m_elevator;
    double m_position;
    double setpoint;

    public ElevatorSet(ElevatorSubsystem elevator, double position) {
        addRequirements(elevator);
        m_elevator = elevator;
        m_position = position;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // m_elevator.setManualOverride(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_elevator.setHeight(m_position);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_elevator.atHeight();
    }
}
