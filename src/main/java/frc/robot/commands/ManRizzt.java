// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ManRizzt extends Command {

    ShooterSubsystem shooterSubsystem;
    LEDSubsystem ledSubsystem;
    private DoubleSupplier speed;
    BooleanSupplier setAngle;

    public ManRizzt(ShooterSubsystem subsystem, LEDSubsystem ledSubsystem, DoubleSupplier speed, BooleanSupplier setAngle) {
        // Use addRequirements() here to declare subsystem dependencies.
        shooterSubsystem = subsystem;
        this.ledSubsystem = ledSubsystem;
        this.setAngle = setAngle;
        this.speed = speed;
        SmartDashboard.putNumber("Set Wrist Angle", 0);
        addRequirements(shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speedValue = speed.getAsDouble() * 0.5;
        
        speedValue = Math.pow(speed.getAsDouble(), 3);
        if (setAngle.getAsBoolean()){
            shooterSubsystem.setWristPosition(SmartDashboard.getNumber("Set Wrist Angle", 0));
        } else {
            shooterSubsystem.manualWristSpeed(speedValue);
        }
    }

    @Override
    public boolean isFinished() {
        ledSubsystem.ReadyToFire(true);
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.manualWristSpeed(0);
        ledSubsystem.ReadyToFire(false);
    }
}
