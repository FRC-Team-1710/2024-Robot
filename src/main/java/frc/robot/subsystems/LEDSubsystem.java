// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  public DigitalOutput AllianceColor = new DigitalOutput(2);
  public DigitalOutput FoundRing = new DigitalOutput(3);
  public DigitalOutput Aimed = new DigitalOutput(4);
  public DigitalOutput RingInIntake = new DigitalOutput(5);
  public DigitalOutput RingOuttaked = new DigitalOutput(6);

  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {}

  @Override
  public void periodic() {}

  public void setAllianceColor() {
    AllianceColor.set(swerveSubsystem.checkRedAlliance().getAsBoolean()); // True is Red, False is Blue
  }

  public void FoundRing(boolean foundRing) {
    FoundRing.set(foundRing);
  }

  public void Aimed(boolean aimed) {
    Aimed.set(aimed);
  }

  public void RingInIntake(boolean ringInIntake) {
    RingInIntake.set(ringInIntake);
  }

  public void RingOuttaked(boolean ringOuttaked) {
    RingOuttaked.set(ringOuttaked);
  }
}
