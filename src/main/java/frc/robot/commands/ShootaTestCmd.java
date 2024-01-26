// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootaTest;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.FiringSolutions;

import java.util.function.BooleanSupplier;

public class ShootaTestCmd extends Command {
  BooleanSupplier Y, A;
  private ShootaTest m_shootaTest;
  private SwerveSubsystem m_SwerveSubsystem;

  /** Creates a new ShootaTestCmd. */
  public ShootaTestCmd(ShootaTest shootaTest, SwerveSubsystem swerveSubsystem) {

    m_shootaTest = shootaTest;
    m_SwerveSubsystem = swerveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootaTest);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shootaTest.SetVelocityFromDashboard();
    m_shootaTest.WristAngleSetFromSmartDashboard();

    m_shootaTest.PointShoot(
        FiringSolutions.getShooterAngle(
            FiringSolutions.getShooterVelocityX(
                m_SwerveSubsystem.getEstimatedPosition().getX(),
                m_SwerveSubsystem.getEstimatedPosition().getY()),
            FiringSolutions.getShooterVelocityZ(),
            FiringSolutions.getRobotVelocityTowardsSpeaker(
                m_SwerveSubsystem.getChassisSpeeds().vxMetersPerSecond,
                m_SwerveSubsystem.getChassisSpeeds().vyMetersPerSecond,
                FiringSolutions.getAngleToSpeaker(
                    m_SwerveSubsystem.getEstimatedPosition().getX(),
                    m_SwerveSubsystem.getEstimatedPosition().getY()),
                m_SwerveSubsystem.getPose().getRotation().getRadians())),
        FiringSolutions.getShooterVelocity(
            FiringSolutions.getShooterVelocityX(
                m_SwerveSubsystem.getEstimatedPosition().getX(),
                m_SwerveSubsystem.getEstimatedPosition().getY()),
            FiringSolutions.getShooterVelocityZ(),
            FiringSolutions.getRobotVelocityTowardsSpeaker(
                m_SwerveSubsystem.getChassisSpeeds().vxMetersPerSecond,
                m_SwerveSubsystem.getChassisSpeeds().vyMetersPerSecond,
                FiringSolutions.getAngleToSpeaker(
                    m_SwerveSubsystem.getEstimatedPosition().getX(),
                    m_SwerveSubsystem.getEstimatedPosition().getY()),
                m_SwerveSubsystem.getPose().getRotation().getRadians()),
            FiringSolutions.getRobotVelocityPerpendicularToSpeaker(
                m_SwerveSubsystem.getChassisSpeeds().vxMetersPerSecond,
                m_SwerveSubsystem.getChassisSpeeds().vyMetersPerSecond,
                FiringSolutions.getAngleToSpeaker(
                    m_SwerveSubsystem.getEstimatedPosition().getX(),
                    m_SwerveSubsystem.getEstimatedPosition().getY()),
                m_SwerveSubsystem.getPose().getRotation().getRadians())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
