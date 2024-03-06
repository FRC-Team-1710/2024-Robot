// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ZeroRizz extends Command {
    ShooterSubsystem shooter;
    public final Timer timer = new Timer();

    /** Creates a new ZeroWrist. */
    public ZeroRizz(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setManualWristSpeed(.45);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.resetWristEncoders(Constants.Shooter.angleOffsetAuto);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() { // Shooter should be hitting hard stop or reach timeout
        if (shooter.isWristMotorStalled() || timer.get() >= 1.5) {
            return true;
        } else {
            return false;
        }
    }
}
