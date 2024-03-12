// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ManRizzt extends Command {

    ShooterSubsystem m_shooterSubsystem;
    LEDSubsystem ledSubsystem;
    private DoubleSupplier speed;
    BooleanSupplier setAngle;
    double lastWristSetpoint = 0.0;
    boolean wristIsLocked = false;

    public ManRizzt(ShooterSubsystem subsystem, LEDSubsystem ledSubsystem, DoubleSupplier speed, BooleanSupplier setAngle) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_shooterSubsystem = subsystem;
        this.ledSubsystem = ledSubsystem;
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
        double speedValue = MathUtil.applyDeadband(speed.getAsDouble(), Constants.stickDeadband);
        speedValue = Math.pow(speedValue, 3);

        if (setAngle.getAsBoolean()) {
            m_shooterSubsystem.setWristByAngle(SmartDashboard.getNumber("Set Wrist Angle", 0));
        } else {
            if (Math.abs(speedValue) > .0) {
                wristIsLocked = false;
                m_shooterSubsystem.setWristSpeedManual(speedValue);
            } else {
                if (m_shooterSubsystem.isZeroed){
                    if (!wristIsLocked){
                        m_shooterSubsystem.setWristByAngle(m_shooterSubsystem.getCurrentShooterAngle());
                        wristIsLocked = true;
                    }
                } else {
                    m_shooterSubsystem.setWristSpeedManual(0);
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        ledSubsystem.ReadyToFire(true);
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //m_shooterSubsystem.setWristSpeedManual(0);
    }
}
