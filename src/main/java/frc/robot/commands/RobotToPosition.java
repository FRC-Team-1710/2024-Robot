// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotToPosition extends Command {

    // Constants such as camera and target height stored
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(16.5);
    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(1.5);

    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(1);

    // PID Controller init
    PIDController forwardController, turnController;

    // Subsystems
    public SwerveSubsystem swerveSubsystem;
    public VisionSubsystem visionSubsystem;

    public RobotToPosition(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;

        // PID Controllers
        forwardController = new PIDController(Constants.Swerve.driveKP, 0, Constants.Swerve.driveKD);
        turnController = new PIDController(Constants.Swerve.angleKP, 0, Constants.Swerve.angleKD);

        addRequirements(swerveSubsystem, visionSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Translation2d forwardSpeed;
        double rotationSpeed;

        var result = visionSubsystem.getLatestResultN();

        if (result.hasTargets()) {
            // First calculate range
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS,
                    TARGET_HEIGHT_METERS,
                    CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));

            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            forwardSpeed = new Translation2d(-forwardController.calculate(range, GOAL_RANGE_METERS), 0);

            // Also calculate angular power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        } else {
            // If we have no targets, stay still.
            forwardSpeed = new Translation2d(0, 0);
            rotationSpeed = 0;
        }

        // Use our forward/turn speeds to control the drivetrain
        swerveSubsystem.drive(forwardSpeed, rotationSpeed, false, false);
    }

}
