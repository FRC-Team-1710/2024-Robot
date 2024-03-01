// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ToBreakOrNotToBreak extends Command {
    IntexerSubsystem intexer;
    LEDSubsystem ledSubsystem;

    /** Creates a new ToBreakOrNotToBreak. */
    public ToBreakOrNotToBreak(IntexerSubsystem intexer, LEDSubsystem ledSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intexer = intexer;
        this.ledSubsystem = ledSubsystem;
        addRequirements(intexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!intexer.shooterBreak()){
            intexer.setShooterIntake(.5);
            ledSubsystem.hasNote = false;
        } else {
            intexer.setShooterIntake(0);
            ledSubsystem.hasNote = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intexer.setShooterIntake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
