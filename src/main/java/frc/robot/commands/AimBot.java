// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.math.FiringSolutionsV3;
import frc.robot.Constants;
import frc.robot.subsystems.IntexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AimBot extends Command {

    private ShooterSubsystem shooter;
    private SwerveSubsystem swerveSubsystem;
    private IntexerSubsystem intexer;
    private PIDController rotationPID = new PIDController(0.65, 0.00001, 0.04);
    private double speed;
    private Timer timer = new Timer();
    private Timer cryAboutIt = new Timer();
    private boolean shootAnyway;

    /** Creates a new AimBot. */
    public AimBot(
            ShooterSubsystem shooterSubsystem,
            SwerveSubsystem swerve,
            IntexerSubsystem intexer,
            double speed) {
        this.shooter = shooterSubsystem;
        this.speed = speed;
        this.intexer = intexer;
        this.swerveSubsystem = swerve;

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(shooterSubsystem, swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        cryAboutIt.reset();
        cryAboutIt.start();
        shooter.setShooterVelocity(speed);
        shootAnyway = !intexer.shooterBreak();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d pose = swerveSubsystem.getPose();
        ChassisSpeeds currentSpeed = swerveSubsystem.getChassisSpeeds();

        double rotationVal = rotationPID.calculate(
                pose.getRotation().getRadians(),
                FiringSolutionsV3.getAngleToMovingTarget(
                        pose.getX(),
                        pose.getY(),
                        FiringSolutionsV3.speakerTargetX,
                        FiringSolutionsV3.speakerTargetY,
                        currentSpeed.vxMetersPerSecond,
                        currentSpeed.vyMetersPerSecond,
                        pose.getRotation().getRadians()));

        swerveSubsystem.drive(
                new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                true,
                false);

        if ((shooter.isShooterAtSpeed() && rotationPID.getPositionError() <= .01710) // real
                && cryAboutIt.get() > 0.5) {
            timer.reset();
            timer.start();
            intexer.setShooterIntake(Constants.Shooter.shooterOutakeSpeed);
        } else if (cryAboutIt.get() >= 2) {
            intexer.setShooterIntake(Constants.Shooter.shooterOutakeSpeed);
        }

        shooter.setWristByAngle(shooter.getCalculatedAngleToSpeaker());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        intexer.setShooterIntake(0);
        shooter.setWristSpeedManual(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (!intexer.shooterBreak() && timer.get() > .2) || cryAboutIt.get() > 3;
    }
}
