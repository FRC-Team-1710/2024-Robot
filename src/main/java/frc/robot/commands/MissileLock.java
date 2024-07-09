// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.math.FiringSolutionsV3;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class MissileLock extends Command {
    private ShooterSubsystem shooter;
    private String target;
    private double firingSpeed;

    /** Creates a new MissileLock. */
    public MissileLock(ShooterSubsystem shooterSub, String target) {
        shooter = shooterSub;
        this.target = target;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!SwerveSubsystem.demoMode){
            firingSpeed = FiringSolutionsV3.convertToRPM(shooter.getCalculatedVelocity());
        } else {
            firingSpeed = Constants.Shooter.idleSpeedRPM;
        }

        if (target == "amp") {
            if (shooter.outsideAllianceWing || SwerveSubsystem.demoMode) {
                shooter.PointShoot(
                        Math.toRadians(58),
                        firingSpeed);
            } else {
                shooter.setShooterVelocity(Constants.Shooter.idleSpeedRPM);
            }
        } else {
            shooter.PointShoot(
                    shooter.getCalculatedAngleToSpeaker(),
                    firingSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // shooter.setShooterVelocity(Constants.Shooter.idleSpeedRPM);
        if (target != "amp" || (shooter.outsideAllianceWing || SwerveSubsystem.demoMode)) {
            shooter.setWristByAngle(Constants.Shooter.intakeAngleRadians);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
