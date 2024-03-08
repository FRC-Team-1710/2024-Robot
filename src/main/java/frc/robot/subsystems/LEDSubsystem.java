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

  // Output bits to the LEDs
  public DigitalOutput[] bits = {bit1, bit2, bit3, bit4}; // Actual Outputs
  boolean[] output = new boolean[4]; // Computed Outputs

  // Booleans used for Easy input
  public Boolean hasNote = false;
  public Boolean chargingOuttake = false;
  public Boolean atSpeed = false; 

  public Boolean[] inputBooleans = {
    false, // Amp             -0 Rainbow Pattern #1 
    false, // Source          -1 Rainbow Pattern #2
    false, // Climb           -2 Rainbow Pattern #3
    false, // Blank/DC        -3 None
    false, // Note Detected   -4 White Solid 
    false, // Charging        -5 Green Pulse (HasNote)
    false, // At Speed        -6 Green BLink (HasNote)
    false, // Charging        -7 Magenta Pulse (NoNote)
    false, // At Speed        -8 Magenta BLink (NoNote)
    false, // Note in Intake  -9 Orange Blink
    false, // Note in Shooter -10 Orange Solid
    false, // Alliance Color  -11 Red Pulse
    false  // Alliance Color  -12 Blue Pulse
  };

  // Use subsystems
  VisionSubsystem vision;
  ShooterSubsystem shooter;
  IntexerSubsystem intexer;
  SwerveSubsystem swerve;
  

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(VisionSubsystem m_VisionSubsystem, ShooterSubsystem m_ShooterSubsystem, IntexerSubsystem m_IntexerSubsystem, SwerveSubsystem m_SwerveSubsystem) {
    this.vision = m_VisionSubsystem;
    this.shooter = m_ShooterSubsystem;
    this.intexer = m_IntexerSubsystem;
    this.swerve = m_SwerveSubsystem;
  }

  @Override
  public void periodic() {
    set(); // Decimal Phase
    encoder(); // Transition Phase
    send(); // Send Phase
  }
  
  private void set() { // Decimal phase
    var results = vision.getLatestResultN();

    // Note Detected
    if (results.hasTargets()) {
      NoteDetected(true);
    } else {
      NoteDetected(false);
    }

    // Driver Station Connected
    if (DriverStation.isDSAttached()) {
      inputBooleans[1] = false;
    } else {
      inputBooleans[1] = true;
    }

    // HasNote for ChargingOuttake and AtSpeed
    if (hasNote) {  // Converts from simple inputs to boolean
      if (chargingOuttake) {
        inputBooleans[5] = true; inputBooleans[6] = false;
      } else if (atSpeed) {
        inputBooleans[5] = false; inputBooleans[6] = true;
      }
      inputBooleans[7] = false; inputBooleans[8] = false;
    } else {
      if (chargingOuttake) {
        inputBooleans[7] = true; inputBooleans[8] = false;
      } else if (atSpeed) {
        inputBooleans[7] = false; inputBooleans[8] = true;
      }
      inputBooleans[5] = false; inputBooleans[6] = false;
    }

    // Check if pathfinding
    if (SwerveSubsystem.followingPath) {
      inputBooleans[0] = true;
    } else {
      inputBooleans[0] = false;
    }

    // Check beam breaks
    if (intexer.intakeBreak()) {
      inputBooleans[9] = true;
      inputBooleans[10] = false;
    } else if (intexer.shooterBreak()) {
      inputBooleans[9] = false;
      inputBooleans[10] = true;
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
    
    String binaryString = Integer.toBinaryString(trueIndex); // Cast Number to Binary
    int length = binaryString.length(); // Find length of Binary (How many bits)
    String finalString; // Final binary string

    // Adds the amount of 0's needed for acurate transcription. Ex. 0x01 -> 0x0001
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
  }

  private void send() { // Send Phase - "Redundancy is bliss"
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
    this.hasNote = hasNote;
  }

  public void ChargingOuttake(boolean chargingOuttake) {
    this.chargingOuttake = chargingOuttake;
  }

  public void ReadyToFire(boolean fire) {
    this.atSpeed = fire;
  }

  public void NoteInIntake(boolean noteInIntake) {
    inputBooleans[9] = noteInIntake;
  }

  public void NoteInShooter(boolean noteInShooter) {
    inputBooleans[10] = noteInShooter;
  }

  public void NoteDetected(boolean note) {
    inputBooleans[4] = note;
  }

  public void setAllianceColor() {
    inputBooleans[11] = Robot.getAlliance();  // True is Red
    inputBooleans[12] = !Robot.getAlliance(); // False is Blue
  }

  public void Disconnected(boolean disconnected) { // NOT CONFIRMED TO BE IN FINAL
    inputBooleans[3] = disconnected;
  }
}
