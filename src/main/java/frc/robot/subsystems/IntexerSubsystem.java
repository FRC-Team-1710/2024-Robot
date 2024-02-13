// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntexerSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */

    private CANSparkBase left = new CANSparkMax(30, MotorType.kBrushless);
    private CANSparkBase right = new CANSparkMax(31, MotorType.kBrushless);
    private DigitalInput breakingBeam = new DigitalInput(6);
    private DigitalInput beamKamen = new DigitalInput(5);

    public IntexerSubsystem() {

    }

    public void set(double speed) {
        left.set(speed);
        right.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Beam Break", breakingBeam.get());
        SmartDashboard.putBoolean("Shooter Beam Break", beamKamen.get());
    }
}
