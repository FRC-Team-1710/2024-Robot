// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntexerSubsystem extends SubsystemBase {

    // Devices
    private CANSparkBase left;
    private CANSparkBase right;
    private CANSparkBase shooterIntake;
    // private TalonFX shooterIntake;
    public boolean intakeThroughShooterPart2isReady = false;
    public boolean resetNoteInShooterPart2isReady = false;

    /** Front Intake */
    private DigitalInput breakingBeam;
    /** Shooter Intake */
    private DigitalInput beamKamen;

    public IntexerSubsystem() {
        left = new CANSparkMax(30, MotorType.kBrushless);
        right = new CANSparkMax(31, MotorType.kBrushless);
        shooterIntake = new CANSparkMax(11, MotorType.kBrushless);

        // Me when we switch to Falcon mid comp then switch back
        // shooterIntake = new TalonFX(32);
        // TalonFXConfiguration configs = new TalonFXConfiguration();
        // configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // shooterIntake.getConfigurator().apply(configs);

        beamKamen = new DigitalInput(1);
        breakingBeam = new DigitalInput(2);

        left.setIdleMode(IdleMode.kCoast);
        right.setIdleMode(IdleMode.kCoast);
        shooterIntake.setIdleMode(IdleMode.kCoast);
        shooterIntake.setInverted(true);

        left.burnFlash();
        right.burnFlash();
        shooterIntake.burnFlash();

        SmartDashboard.putData(this);
    }

    public void setALL(double speed) {
        if (speed == 0) {
            left.stopMotor();
            right.stopMotor();
            shooterIntake.stopMotor();
        } else {
            left.set(speed);
            right.set(speed);
            shooterIntake.set(speed);
        }
    }

    public void setFrontIntake(double speed) {
        if (speed == 0) {
            left.stopMotor();
            right.stopMotor();
        } else {
            left.set(speed);
            right.set(speed);
        }
    }

    public void setShooterIntake(double speed) {
        if (speed == 0) {
            shooterIntake.stopMotor();
        } else {
            shooterIntake.set(speed);
        }
    }

    public boolean intakeBreak() {
        return !breakingBeam.get();
    }

    public boolean shooterBreak() {
        return !beamKamen.get();
    }

    public void setIntakeThroughShooterPart2Status(boolean value) {
        intakeThroughShooterPart2isReady = value;
    }

    public void resetNoteInShooterPart2Status(boolean value) {
        resetNoteInShooterPart2isReady = value;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Beam Break", intakeBreak());
        SmartDashboard.putBoolean("Shooter Beam Break", shooterBreak());

        SmartDashboard.putNumber("Intake Left Current", left.getOutputCurrent());
        SmartDashboard.putNumber("Intake Right Current", right.getOutputCurrent());
        SmartDashboard.putNumber("Shooter Intake Current", shooterIntake.getOutputCurrent());
    }
}
