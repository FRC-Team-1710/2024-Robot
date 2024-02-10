// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntexerSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */

    TalonFX left = new TalonFX(30);
    TalonFX right = new TalonFX(31);
    DigitalInput intakeBeamBreak = new DigitalInput(0);

    public IntexerSubsystem() {

    }

    public void set(double speed) {
        left.set(speed);
        right.set(-speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Beam Break", intakeBeamBreak.get());
    }
}
