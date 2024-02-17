// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.FiringSolutionsV2;
import frc.robot.subsystems.IntexerSubsystem;

public class IntexBestHex extends Command {

    IntexerSubsystem intexer;
    boolean in;

    /** Creates a new IntexBestHex. */
    public IntexBestHex(IntexerSubsystem intexer, boolean in) {
        this.in = in;
        this.intexer = intexer;
        addRequirements(intexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        FiringSolutionsV2.resetR();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (in) { // Hood logic to run forwards or backwards
            if (intexer.intakeBreak() && !intexer.shooterBreak()) { // If note is not at shooter yet
                intexer.setALL(.35);
            } else if (!intexer.intakeBreak() && intexer.shooterBreak()) { // Stop note if at shooter
                intexer.setALL(0);
            } else { // Note is not in robot
                intexer.setFrontIntake(.75);
            }
        } else { // Outtake
            intexer.setALL(-0.5);
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
        return false;
    }
}
