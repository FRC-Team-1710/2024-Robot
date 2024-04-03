// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class WheelDiameter extends Command {
  /** Creates a new WheelDiameter. */
 
 public Pigeon2 gyro;
 public SwerveSubsystem swerveSubsystem;
 private Timer timer = new Timer();
 double beginningGyro;
 double finalGyro;
 double gyroDelta;
 double[] beginningMotorPose;
 double[] finalMotorPose;
 ChassisSpeeds rotationSpeed = new ChassisSpeeds(0, 0, Math.PI/2);
 

  public WheelDiameter(SwerveSubsystem swerveSubsystem) {
   this.swerveSubsystem = swerveSubsystem;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    beginningGyro = swerveSubsystem.getGyroYaw().getRadians();
    beginningMotorPose = swerveSubsystem.getModulePositionsRadians();
    
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   swerveSubsystem.setChassisSpeeds(rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.setChassisSpeeds(new ChassisSpeeds());
    finalGyro = swerveSubsystem.getGyroYaw().getRadians();
    finalMotorPose = swerveSubsystem.getModulePositionsRadians();

    double wheelRadius0 = ((beginningGyro - finalGyro) * Constants.Swerve.trackWidth) / (beginningMotorPose[0] - finalMotorPose[0]);
    double wheelRadius1 = ((beginningGyro - finalGyro) * Constants.Swerve.trackWidth) / (beginningMotorPose[1] - finalMotorPose[1]);
    double wheelRadius2 = ((beginningGyro - finalGyro) * Constants.Swerve.trackWidth) / (beginningMotorPose[2] - finalMotorPose[2]);
    double wheelRadius3 = ((beginningGyro - finalGyro) * Constants.Swerve.trackWidth) / (beginningMotorPose[3] - finalMotorPose[3]);

    SmartDashboard.putNumber("Wheel Radius0", wheelRadius0);
    SmartDashboard.putNumber("Wheel Radius1", wheelRadius1);
    SmartDashboard.putNumber("Wheel Radius2", wheelRadius2);
    SmartDashboard.putNumber("Wheel Radius3", wheelRadius3);
    DataLogManager.log("Wheel Radius0" + wheelRadius0);
    DataLogManager.log("Wheel Radius1" + wheelRadius1);
    DataLogManager.log("Wheel Radius2" + wheelRadius2);
    DataLogManager.log("Wheel Radius3" + wheelRadius3);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 10;
  }
}