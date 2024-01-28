// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootaTest;

public class ManRizzt extends Command {

    ShootaTest shooterSubsystem;
    private double speed;

    public ManRizzt(ShootaTest subsystem, double speed) {
        // Use addRequirements() here to declare subsystem dependencies.
        shooterSubsystem = subsystem;
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
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        //shooterSubsystem.manualWristSpeed(0);
    }
}
