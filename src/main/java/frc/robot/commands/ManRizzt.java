// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ManRizzt extends Command {

    ShooterSubsystem shooterSubsystem;
    LEDSubsystem ledSubsystem;
    private double speed;

    public ManRizzt(ShooterSubsystem subsystem, LEDSubsystem ledSubsystem, double speed) {
        // Use addRequirements() here to declare subsystem dependencies.
        shooterSubsystem = subsystem;
        this.ledSubsystem = ledSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSubsystem.manualWristSpeed(speed);
        ledSubsystem.ReadyToFire(false);
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
