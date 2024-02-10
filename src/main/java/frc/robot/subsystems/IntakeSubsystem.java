// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  CANcoder m_intakeEncoder = new CANcoder(3);
  CANcoder m_intakeEncodera = new CANcoder(1);

  TalonFX m_motor = new TalonFX(4);
  TalonFX m_motora = new TalonFX(2);

  public IntakeSubsystem() {}

  public void set(double speed) {
    m_motor.set(speed);
    m_motora.set(-speed);
  }

  @Override
  public void periodic() {
   
    // This method will be called once per scheduler run
  }
}
