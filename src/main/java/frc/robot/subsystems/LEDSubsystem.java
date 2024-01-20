// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  public DigitalOutput AllianceColor = new DigitalOutput(2); // Alliance color
  public DigitalOutput FoundNote = new DigitalOutput(3);     // Sends if the camera 'OnionRing' sees a note
  public DigitalOutput Aimed = new DigitalOutput(4);         // Sends if the outtake is aimed
  public DigitalOutput NoteInIntake = new DigitalOutput(5);  // Sends if the Note is in the intake/outtake
  public DigitalOutput NoteOuttaked = new DigitalOutput(6);  // Sends if the Note has been outtaked

  VisionSubsystem vision;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(VisionSubsystem m_VisionSubsystem) {
    this.vision = m_VisionSubsystem;
  }

  @Override
  public void periodic() {
    var results = vision.getLatestResultN();

    if (results.hasTargets()) {
      FoundNote(true);
    } else {
      FoundNote(false);
    }
  }

  public BooleanSupplier checkRedAlliance() {
        var alliance = DriverStation.getAlliance(); // Have to use var because of the optional container
        if (alliance.isPresent()) {
            return () -> alliance.get() == DriverStation.Alliance.Red;
        }
        return () -> false;
    }

  public void setAllianceColor() {
    AllianceColor.set(checkRedAlliance().getAsBoolean()); // True is Red, False is Blue
  }

  public void FoundNote(boolean foundNote) {
    FoundNote.set(foundNote);
  }

  public void Aimed(boolean aimed) {
    Aimed.set(aimed);
  }

  public void NoteInIntake(boolean noteInIntake) {
    NoteInIntake.set(noteInIntake);
  }

  public void NoteOuttaked(boolean noteOuttaked) {
    NoteOuttaked.set(noteOuttaked);
  }
}
