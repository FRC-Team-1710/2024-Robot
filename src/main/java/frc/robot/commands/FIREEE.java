// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.FiringSolutions;
import frc.robot.subsystems.IntexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FIREEE extends Command {
    private ShooterSubsystem m_shootaTest;
    private IntexerSubsystem intexer;

    public FIREEE(ShooterSubsystem shootaTest, IntexerSubsystem intex) {
        m_shootaTest = shootaTest;
        intexer = intex;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shootaTest, intex);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_shootaTest.shooterAtSpeed()){
            intexer.setShooterIntake(.75);
        }
        m_shootaTest.PointShoot(m_shootaTest.getCalculatedAngle(), FiringSolutions.convertToRPM(m_shootaTest.getCalculatedVelocity()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intexer.setShooterIntake(0);
        m_shootaTest.SetShooterVelocity(0);
        m_shootaTest.manualWristSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
