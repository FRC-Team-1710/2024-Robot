// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorSubsystem;

public class ElevatorCmd extends Command {
    private elevatorSubsystem m_elevator;
   double m_position;
    double setpoint;
  /** Creates a new ElevatorCmd. */
  public ElevatorCmd(elevatorSubsystem elevate, double position) {
   addRequirements(elevate);
   m_elevator = elevate;
   m_position = position;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
    return false;
  }
}
