// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FIREEFORACERTAINAMOUNTOFTIME extends Command {
    private ShooterSubsystem shooter;
    private IntexerSubsystem intexer;
    private double time;
    private Timer timer = new Timer();

    public FIREEFORACERTAINAMOUNTOFTIME(
            ShooterSubsystem shooterSub, IntexerSubsystem intex, double time) {
        shooter = shooterSub;
        intexer = intex;
        this.time = time;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intex);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intexer.setShooterIntake(.9);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intexer.setShooterIntake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() > time;
    }
}
