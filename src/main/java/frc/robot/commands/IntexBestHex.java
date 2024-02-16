// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (in) {
            if (intexer.intakeBreak() && !intexer.shooterBreak()) {
                intexer.setALL(.35);
            } else if (!intexer.intakeBreak() && intexer.shooterBreak()) {
                intexer.setALL(0);
            } else {
                intexer.setFrontIntake(.75);
            }
        } else {
            intexer.setALL(-0.5);
        }

        //intexer.setALL(.75);
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
