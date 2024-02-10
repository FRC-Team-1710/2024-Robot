// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LEDSubsystem extends SubsystemBase {
  public DigitalOutput bit1 = new DigitalOutput(2); // Bit 1 (1)
  public DigitalOutput bit2 = new DigitalOutput(3); // Bit 2 (2)
  public DigitalOutput bit3 = new DigitalOutput(4); // Bit 3 (4)
  public DigitalOutput bit4 = new DigitalOutput(5); // Bit 4 (8)

  public DigitalOutput[] bits = {bit1, bit2, bit3, bit4};

  public Boolean[] inputBooleans = {
    false, // AllianceColor   0
    false, // Disconnected    1
    false, // NoteDetected    2
    false, // Intaking        3
    false, // ChargingOuttake 4
    false  // ReadyToFire     5 
  };

  boolean[] output = new boolean[4];

  VisionSubsystem vision;
  ShooterSubsystem shooter;
  

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(VisionSubsystem m_VisionSubsystem, ShooterSubsystem m_ShooterSubsystem) {
    this.vision = m_VisionSubsystem;
    this.shooter = m_ShooterSubsystem;
  }

  @Override
  public void periodic() {
    set(); // Set booleans
    encoder(); // Encode the values
    send(); // Output to the pins
  }
  
  private void set() {
    var results = vision.getLatestResultN();

    if (results.hasTargets()) {
      NoteDetected(true);
    } else {
      NoteDetected(false);
    }

    if (DriverStation.isDSAttached()) {
      inputBooleans[1] = false;
    } else {
      inputBooleans[1] = true;
    }

    if (shooter.getVelocity() > shooter.getCalculatedVelocity() - 0.5) {
      ReadyToFire(true);
    } else if (shooter.getVelocity() > 0.5 && shooter.getCalculatedVelocity() > shooter.getVelocity()) {
      ReadyToFire(false);
      ChargingOuttake(true);
    } else {
      ReadyToFire(false);
      ChargingOuttake(false);
    }
  }

  private void encoder() {
    // Listed in order of priority
    if (inputBooleans[5]) { // Ready to fire
      output[0] = true; output[1] = false; output[2] = true; output[3] = false; // 1010
    } else if (inputBooleans[3]) { // Intaking
      output[0] = true; output[1] = true; output[2] = false; output[3] = false; // 1100
    } else if (inputBooleans[4]) { // Charging the Outtake
      output[0] = false; output[1] = false; output[2] = true; output[3] = false; // 0010
    } else if (inputBooleans[2]) { // Note Detected
      output[0] = false; output[1] = true; output[2] = false; output[3] = false; // 0100
    } else if (inputBooleans[1]) { // Driver Station Disconnected
      output[0] = true; output[1] = false; output[2] = false; output[3] = false; // 1000
    } else if (inputBooleans[0]) { // Alliance color
      output[0] = false; output[1] = false; output[2] = false; output[3] = false; // 0000
    }
  }

  private void send() { // Redundant
    for (int i = 0; i < output.length; i++) {
      bits[i].set(output[i]);
    }
  }

  public void setAllianceColor() {
    inputBooleans[0] = Robot.getAlliance(); // True is Red, False is Blue
  }

  public void Disconnected(boolean disconnected) {
    inputBooleans[1] = disconnected;
  }

  public void NoteDetected(boolean note) {
    inputBooleans[2] = note;
  }

  public void Intaking(boolean intaking) {
    inputBooleans[3] = intaking;
  }

  public void ChargingOuttake(boolean chargingOuttake) {
    inputBooleans[4] = chargingOuttake;
  }

  public void ReadyToFire(boolean fire) {
    inputBooleans[5] = fire;
  }
}
