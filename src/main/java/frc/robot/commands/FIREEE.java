// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootaTest;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.FiringSolutions;

public class FIREEE extends Command {
    private ShootaTest m_shootaTest;
    private SwerveSubsystem m_SwerveSubsystem;

    /** Creates a new ShootaTestCmd. */
    public FIREEE(ShootaTest shootaTest, SwerveSubsystem swerveSubsystem) {

        m_shootaTest = shootaTest;
        m_SwerveSubsystem = swerveSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shootaTest);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // m_shootaTest.SetVelocityFromDashboard();
        // m_shootaTest.WristAngleSetFromSmartDashboard();

        Pose2d pose = m_SwerveSubsystem.getPose();
        double rotation = (pose.getRotation().getRadians() + Math.PI) % 2 * Math.PI;

        ChassisSpeeds chassisSpeeds = m_SwerveSubsystem.getChassisSpeeds();
        double shooterAngle = FiringSolutions.getShooterAngle(
                pose.getX(),
                pose.getY(),
                FiringSolutions.getRobotVelocityTowardsSpeaker(
                        chassisSpeeds.vxMetersPerSecond,
                        chassisSpeeds.vyMetersPerSecond,
                        FiringSolutions.getAngleToSpeaker(
                                pose.getX(),
                                pose.getY()),
                        rotation));
        double shooterVelocity = FiringSolutions.getShooterVelocity(
                FiringSolutions.getShooterVelocityX(pose.getX(), pose.getY()),
                FiringSolutions.getShooterVelocityZ(),
                FiringSolutions.getRobotVelocityTowardsSpeaker(
                        chassisSpeeds.vxMetersPerSecond,
                        chassisSpeeds.vyMetersPerSecond,
                        FiringSolutions.getAngleToSpeaker(
                                pose.getX(),
                                pose.getY()),
                        pose.getRotation().getRadians()),
                FiringSolutions.getRobotVelocityPerpendicularToSpeaker(
                        chassisSpeeds.vxMetersPerSecond,
                        chassisSpeeds.vyMetersPerSecond,
                        FiringSolutions.getAngleToSpeaker(
                                pose.getX(),
                                pose.getY()),
                        pose.getRotation().getRadians()));

        m_shootaTest.PointShoot(shooterAngle, FiringSolutions.convertToRPM(shooterVelocity));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shootaTest.SetShooterVelocity(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
