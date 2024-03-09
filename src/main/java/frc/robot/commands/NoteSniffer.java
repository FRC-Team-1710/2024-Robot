// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class NoteSniffer extends Command {

    private SwerveSubsystem swerveSubsystem;
    private VisionSubsystem vision;
    private IntexerSubsystem intexer;
    private ShooterSubsystem shooter;
    private PIDController rotationPID = new PIDController(0.65, 0.00001, 0.04);
    private Timer timer = new Timer();
    private boolean noteInside = false;
    private double translationVal = 0.5;

    /** Creates a new IntakeWithVision. */
    public NoteSniffer(SwerveSubsystem swerve, VisionSubsystem vision, IntexerSubsystem intexer,
            ShooterSubsystem shooter) {
        this.swerveSubsystem = swerve;
        this.vision = vision;
        this.intexer = intexer;
        this.shooter = shooter;
        addRequirements(swerve, intexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        translationVal = .5;
        noteInside = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setWristByAngle(Constants.Shooter.intakeAngleRadians);

        PhotonPipelineResult result = vision.getLatestResultN();
        double rotationVal;

        if (intexer.intakeBreak()) {
            noteInside = true;
            translationVal = 0;
            rotationVal = 0;
        }

        if (result.hasTargets() && !noteInside) { // Lock robot towards detected note
            double yawToNote = Math.toRadians(result.getBestTarget().getYaw())
                    + swerveSubsystem.getGyroYaw().getRadians();

            SmartDashboard.putNumber("Note Yaw", yawToNote);

            rotationVal = rotationPID.calculate(yawToNote, swerveSubsystem.getGyroYaw().getRadians());

            intexer.setFrontIntake(.8);
            intexer.setShooterIntake(.35);
        } else {
            rotationVal = 0;
        }

        //if (shooter.getDistanceToSpeaker() < 2.5){
            swerveSubsystem.drive(
                    new Translation2d(translationVal, 0).times(Constants.Swerve.maxSpeed),
                    rotationVal * Constants.Swerve.maxAngularVelocity,
                    false,
                    false);
        /* } else {
            intexer.setALL(-.5);
            swerveSubsystem.drive(
                    new Translation2d(-0.2, 0).times(Constants.Swerve.maxSpeed),
                    0 * Constants.Swerve.maxAngularVelocity,
                    false,
                    false);
        }*/
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intexer.setALL(0);
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return intexer.shooterBreak() || timer.get() > 3;
    }
}
