// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntexerSubsystem;

public class IntexForAutosByAutos extends Command {

    private IntexerSubsystem intexer;

    /** Creates a new IntexForAutosByAutos. */
    public IntexForAutosByAutos(IntexerSubsystem intexerSub) {
        this.intexer = intexerSub;
        addRequirements(intexerSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (intexer.intakeBreak() && !intexer.shooterBreak()) { // If note is not at shooter yet
            intexer.setALL(.4);
        } else if (!intexer.intakeBreak() && intexer.shooterBreak()) { // Stop note if at shooter
            intexer.setALL(0);
        } else { // Note is not in robot
            intexer.setFrontIntake(.75);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!intexer.intakeBreak() && intexer.shooterBreak()) {
            return true;
        } else {
            return false;
        }
    }
}
