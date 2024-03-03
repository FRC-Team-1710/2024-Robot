// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntexForAutosByAutos extends Command {

    private IntexerSubsystem intexer;
    private ShooterSubsystem shooter;

    /** Creates a new IntexForAutosByAutos. */
    public IntexForAutosByAutos(IntexerSubsystem intexerSub, ShooterSubsystem shooterSubsystem) {
        this.intexer = intexerSub;
        this.shooter = shooterSubsystem;
        addRequirements(intexerSub, shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setWristPosition(Constants.Shooter.intakeAngleRadians);

        if (intexer.intakeBreak() && !intexer.shooterBreak()) { // If note is not at shooter yet
            intexer.setALL(.35);
        } else if (intexer.shooterBreak()) { // Stop note if at shooter
            intexer.setALL(0);
        } else { // Note is not in robot
            intexer.setFrontIntake(.85);
            intexer.setShooterIntake(.35);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intexer.setALL(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (intexer.shooterBreak()) {
            return true;
        } else {
            return false;
        }
    }
}
