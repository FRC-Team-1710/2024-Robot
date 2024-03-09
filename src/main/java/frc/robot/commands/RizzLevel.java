// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RizzLevel extends Command {

    ShooterSubsystem shooter;
    double angle;

    /** Creates a new RizzLevel. */
    public RizzLevel(ShooterSubsystem noteAccelerator, double angleInRADIANS) {
        shooter = noteAccelerator;
        angle = angleInRADIANS;
        addRequirements(noteAccelerator);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.setManualOverride(false);
        shooter.setWristByAngle(angle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //shooter.manualWristSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
