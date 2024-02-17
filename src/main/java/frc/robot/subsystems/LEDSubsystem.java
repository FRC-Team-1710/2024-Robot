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
    false, // Amp             -0 Rainbow Pattern #1
    false, // Source          -1 Rainbow Pattern #2
    false, // Climb           -2 Rainbow Pattern #3
    false, // Has Note        -3 For Computing
    false, // Charging        -4 Green/Magenta Pulse (HasNote)
    false, // At Speed        -5 Green/Magenta BLink (HasNote)
    false, // Note in Intake  -6 Orange Blink
    false, // Note in Shooter -7 Orange Solid
    false, // Note Detected   -8 White Solid 
    false, // Alliance Color  -9 Red/Blue Wave
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
    set(); // Decimal Phase
    encoder(); // Transition Phase
    send(); // Send Phase
  }
  
  private void set() { // Decimal phase
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

  private void encoder() { // Transition phase
    // Decimal to Binary
    int pin_amount = 4; // Amount of pins
        
    int trueIndex = 0; // Index of selected LED sequence

    for (int i = 0; i < inputBooleans.length; i++) { // Picks the first true sequence based on priority
      if (inputBooleans[i]) {
        trueIndex = i;
        break;
      }
    }
    
    String binaryString = Integer.toBinaryString(trueIndex);

    int length = binaryString.length();

    String finalString;
    if (length < pin_amount) {
        int zerosToAdd = pin_amount - length;
        StringBuilder paddedStringBuilder = new StringBuilder();

        for (int i = 0; i < zerosToAdd; i++) {
            paddedStringBuilder.append("0");
        }

        paddedStringBuilder.append(binaryString);
        String paddedBinaryString = paddedStringBuilder.toString();

        finalString = paddedBinaryString;
    } else {
        finalString = binaryString;
    }
    
    for (int i = pin_amount - 1; i >= 0; i--) {
        output[i] = finalString.charAt(i) == '1';
    }
    
    
    // if (inputBooleans[0]) {
    //   output[0] = false; output[1] = false; output[2] = false; output[3] = false; // 0000
    // } else if (inputBooleans[1]) {
    //   output[0] = true; output[1] = false; output[2] = false; output[3] = false;  // 1000
    // } else if (inputBooleans[2]) {
    //   output[0] = false; output[1] = true; output[2] = false; output[3] = false;  // 0100
    // } else if (inputBooleans[3]) {
    //   output[0] = true; output[1] = true; output[2] = false; output[3] = false;   // 1100
    // } else if (inputBooleans[4]) {
    //   output[0] = false; output[1] = false; output[2] = true; output[3] = false;  // 0010
    // } else if (inputBooleans[5]) {
    //   output[0] = true; output[1] = false; output[2] = true; output[3] = false;   // 1010
    // } else if (inputBooleans[6]) {
    //   output[0] = false; output[1] = true; output[2] = true; output[3] = false;   // 0110
    // } else if (inputBooleans[7]) {
    //   output[0] = true; output[1] = true; output[2] = true; output[3] = false;    // 1110
    // } else if (inputBooleans[8]) {
    //   output[0] = false; output[1] = false; output[2] = false; output[3] = true;  // 0001
    // } else if (inputBooleans[9]) {
    //   output[0] = true; output[1] = false; output[2] = false; output[3] = true;   // 1001
    // } else if (inputBooleans[10]) {
    //   output[0] = false; output[1] = true; output[2] = false; output[3] = true;   // 0101
    // } else if (inputBooleans[11]) {
    //   output[0] = true; output[1] = true; output[2] = false; output[3] = true;    // 1101
    // } else if (inputBooleans[12]) {
    //   output[0] = false; output[1] = false; output[2] = true; output[3] = true;   // 0011
    // } else if (inputBooleans[13]) {
    //   output[0] = true; output[1] = false; output[2] = true; output[3] = true;    // 1011
    // } else if (inputBooleans[14]) {
    //   output[0] = false; output[1] = true; output[2] = true; output[3] = true;    // 0111
    // } else if (inputBooleans[15]) {
    //   output[0] = true; output[1] = true; output[2] = true; output[3] = true;     // 1111
    // }
  }

  private void send() { // Send Phase - Redundancy is bliss
    for (int i = 0; i < output.length; i++) {
      bits[i].set(output[i]);
    }
  }

  // SET FUNCTIONS
  public void ampTele(boolean amp) {
    inputBooleans[0] = amp;
  }
  
  public void sourceTele(boolean source) {
    inputBooleans[1] = source;
  }

  public void climbTele(boolean climb) {
    inputBooleans[2] = climb;
  }

  public void HasNote(boolean hasNote) {
    inputBooleans[3] = hasNote;
  }

  public void ChargingOuttake(boolean chargingOuttake) {
    inputBooleans[4] = chargingOuttake;
  }

  public void ReadyToFire(boolean fire) {
    inputBooleans[5] = fire;
  }

  public void NoteInIntake(boolean noteInIntake) {
    inputBooleans[6] = noteInIntake;
  }

  public void NoteInShooter(boolean noteInShooter) {
    inputBooleans[7] = noteInShooter;
  }

  public void NoteDetected(boolean note) {
    inputBooleans[8] = note;
  }

  public void setAllianceColor() {
    inputBooleans[9] = Robot.getAlliance(); // True is Red, False is Blue
  }

  public void Disconnected(boolean disconnected) { // NOT CONFIRMED TO BE IN FINAL
    inputBooleans[10] = disconnected;
  }
}
