// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ManRizzt extends Command {

    ShooterSubsystem m_shooterSubsystem;
    private DoubleSupplier speed;
    BooleanSupplier setAngle;
    double lastWristSetpoint = 0.0;
    boolean wristIsLocked = false;

    public ManRizzt(ShooterSubsystem shooterSubsystem, DoubleSupplier speed, BooleanSupplier setAngle) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_shooterSubsystem = shooterSubsystem;
        this.setAngle = setAngle;
        this.speed = speed;
        SmartDashboard.putNumber("Set Wrist Angle", 0);
        addRequirements(m_shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speedValue = Math.pow(speed.getAsDouble(), 3);

        if (setAngle.getAsBoolean()) {
            m_shooterSubsystem.setWristByAngle(SmartDashboard.getNumber("Set Wrist Angle", 0));
        } else {
            if (Math.abs(speedValue) > .05) {
                wristIsLocked = false;
                m_shooterSubsystem.setManualWristSpeed(speedValue);
            } else {
                if (m_shooterSubsystem.isZeroed){
                    if (!wristIsLocked){
                        m_shooterSubsystem.setWristByAngle(m_shooterSubsystem.getCurrentShooterAngle());
                        wristIsLocked = true;
                    }
                } else {
                    m_shooterSubsystem.setManualWristSpeed(0);
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setManualWristSpeed(0);
    }
}
