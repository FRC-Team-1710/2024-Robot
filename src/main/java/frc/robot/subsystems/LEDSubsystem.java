// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LEDSubsystem extends SubsystemBase {
  public DigitalOutput AllianceColor = new DigitalOutput(2); // Alliance color
  public DigitalOutput Disconnected = new DigitalOutput(3); // Disconnected color
  public DigitalOutput NoteDetected = new DigitalOutput(4); // Note Detected color
  public DigitalOutput Intaking = new DigitalOutput(5); // Intaking color
  public DigitalOutput ChargingOuttake = new DigitalOutput(6); // Charging the Outtake color
  public DigitalOutput ReadyToFire = new DigitalOutput(7); // Ready to Fire color

  VisionSubsystem vision;

  

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(VisionSubsystem m_VisionSubsystem) {
    this.vision = m_VisionSubsystem;
  }

  @Override
  public void periodic() {
    var results = vision.getLatestResultN();

    if (results.hasTargets()) {
      NoteDetected(true);
    } else {
      NoteDetected(false);
    }

    if (DriverStation.isDSAttached()) {
      Disconnected.set(false);
    } else {
      Disconnected.set(true);
    }
  }

  public void setAllianceColor() {
    AllianceColor.set(Robot.getAlliance()); // True is Red, False is Blue
  }

  public void Disconnected(boolean disconnected) {
    Disconnected.set(disconnected);
  }

  public void NoteDetected(boolean note) {
    NoteDetected.set(note);
  }

  public void Intaking(boolean intaking) {
    Intaking.set(intaking);
  }

  public void ChargingOuttake(boolean chargingOuttake) {
    ChargingOuttake.set(chargingOuttake);
  }

  public void ReadyToFire(boolean fire) {
    ReadyToFire.set(fire);
  }
}
