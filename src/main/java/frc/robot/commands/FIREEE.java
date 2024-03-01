// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FIREEE extends Command {
    private ShooterSubsystem shooter;
    private IntexerSubsystem intexer;
    private LEDSubsystem ledSubsystem;

    public FIREEE(ShooterSubsystem shooterSub, IntexerSubsystem intex, LEDSubsystem ledSubsystem) {
        shooter = shooterSub;
        intexer = intex;
        this.ledSubsystem = ledSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intex);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (shooter.shooterAtSpeed()){
            intexer.setShooterIntake(.9);
            ledSubsystem.atSpeed = true;
            ledSubsystem.chargingOuttake = false;
            ledSubsystem.hasNote = false;
        } else {
            ledSubsystem.atSpeed = false;
            ledSubsystem.chargingOuttake = true;
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
