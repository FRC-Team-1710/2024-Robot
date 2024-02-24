package frc.robot.commands;

import frc.lib.math.FiringSolutions;
import frc.lib.math.FiringSolutionsV3;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private SwerveSubsystem swerveSubsystem;
    private VisionSubsystem vision;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier shooterOverrideAmp;
    private BooleanSupplier shooterOverrideSpeaker;
    private BooleanSupplier intakeOverride;
    private PIDController rotationPID = new PIDController(0.65, 0.00001, 0.04);

    public TeleopSwerve(SwerveSubsystem swerve, VisionSubsystem vision, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier shooterOverrideAmp, BooleanSupplier shooterOverrideSpeaker, BooleanSupplier intake) {
        this.swerveSubsystem = swerve;
        this.vision = vision;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.shooterOverrideAmp = shooterOverrideAmp;
        this.shooterOverrideSpeaker = shooterOverrideSpeaker;
        this.intakeOverride = intake;
        SmartDashboard.putData(rotationPID);
    }

    @Override
    public void execute() {
        Pose2d pose = swerveSubsystem.getPose();
        boolean robotCentric = robotCentricSup.getAsBoolean();
        PhotonPipelineResult result = vision.getLatestResultN();

        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal;

        /* Exponential Drive */
        translationVal = Math.copySign(Math.pow(translationVal, 2), translationVal);
        strafeVal = Math.copySign(Math.pow(strafeVal, 2), strafeVal);

        if (shooterOverrideSpeaker.getAsBoolean()) { // Lock robot angle to speaker
            ChassisSpeeds currentSpeed = swerveSubsystem.getChassisSpeeds();

            rotationVal = rotationPID.calculate(swerveSubsystem.getHeading().getRadians(),
                    FiringSolutionsV3.getAngleToMovingTarget(pose.getX(), pose.getY(), FiringSolutionsV3.speakerTargetX, FiringSolutionsV3.speakerTargetY,
                            currentSpeed.vxMetersPerSecond,
                            currentSpeed.vyMetersPerSecond,
                            FiringSolutions.getAngleToSpeaker(pose.getX(), pose.getY())));
                            
        } else if (shooterOverrideAmp.getAsBoolean()) { // Lock robot angle to amp
            ChassisSpeeds currentSpeed = swerveSubsystem.getChassisSpeeds();

            rotationVal = rotationPID.calculate(swerveSubsystem.getHeading().getRadians(),
                    FiringSolutionsV3.getAngleToMovingTarget(pose.getX(), pose.getY(), FiringSolutionsV3.ampTargetX, FiringSolutionsV3.ampTargetY,
                            currentSpeed.vxMetersPerSecond,
                            currentSpeed.vyMetersPerSecond,
                            FiringSolutions.getAngleToSpeaker(pose.getX(), pose.getY())));

        } else if (intakeOverride.getAsBoolean() && result.hasTargets()) { // Lock robot towards detected note
            double yawToNote = Math.toRadians(result.getBestTarget().getYaw()) + swerveSubsystem.getGyroYaw().getRadians();

            SmartDashboard.putNumber("Note Yaw", yawToNote);

            rotationVal = rotationPID.calculate(yawToNote, swerveSubsystem.getGyroYaw().getRadians());
        } else {
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
            rotationVal = Math.copySign(Math.pow(rotationVal, 2), rotationVal);
        }

        if (Robot.getAlliance() && !robotCentric){ // Invert field oriented for always blue origin
            translationVal = -translationVal;
            strafeVal = -strafeVal;
        }

        /* Drive */
        swerveSubsystem.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                !robotCentric,
                true);
    }
}