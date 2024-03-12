// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.FiringSolutionsV3;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class MissileLock extends Command {
    private ShooterSubsystem shooter;
    private String target;

    /** Creates a new MissileLock. */
    public MissileLock(ShooterSubsystem shooterSub, String target) {
        shooter = shooterSub;
        this.target = target;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (target == "amp") {
            if (shooter.getDistanceTo(FiringSolutionsV3.trueAmpX, FiringSolutionsV3.trueAmpY) > 4) {
                shooter.PointShoot(Math.toRadians(55),
                        FiringSolutionsV3.convertToRPM(12));
            } else {
                shooter.setShooterVelocity(FiringSolutionsV3.convertToRPM(12));
            }
        } else {
            shooter.PointShoot(shooter.getCalculatedAngleToSpeaker(),
                    FiringSolutionsV3.convertToRPM(shooter.getCalculatedVelocity()));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //shooter.setShooterVelocity(Constants.Shooter.idleSpeedRPM);
        if (target != "amp" || shooter.getDistanceTo(FiringSolutionsV3.trueAmpX, FiringSolutionsV3.trueAmpY) > 4){
            shooter.setWristByAngle(Constants.Shooter.intakeAngleRadians);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
