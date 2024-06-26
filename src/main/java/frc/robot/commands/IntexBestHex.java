// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.math.FiringSolutionsV3;
import frc.robot.Constants;
import frc.robot.subsystems.IntexerSubsystem;

public class IntexBestHex extends Command {

    IntexerSubsystem intexer;
    boolean in;
    Joystick controller;

    /** Creates a new IntexBestHex. */
    public IntexBestHex(IntexerSubsystem intexer, boolean in, Joystick controller) {
        this.in = in;
        this.intexer = intexer;
        this.controller = controller;
        addRequirements(intexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        FiringSolutionsV3.resetAllR();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (in) { // Hood logic to run forwards or backwards
            if (intexer.intakeBreak() && !intexer.shooterBreak()) { // If note is not at shooter yet
                intexer.setALL(Constants.Intake.noteInsideSpeed);
                controller.setRumble(RumbleType.kBothRumble, 0.5);
            } else if (!intexer.intakeBreak()
                    && intexer.shooterBreak()) { // Stop note if at shooter
                controller.setRumble(RumbleType.kBothRumble, 0.75);
                intexer.setALL(0);
            } else { // Note is not in robot
                intexer.setFrontIntake(Constants.Intake.noteOutsideSpeed);
            }
        } else { // Outtake
            intexer.setALL(Constants.Intake.outakeSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intexer.setALL(0);
        controller.setRumble(RumbleType.kBothRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
