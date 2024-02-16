package frc.robot.commands;

import frc.lib.math.FiringSolutions;
import frc.lib.math.FiringSolutionsV2;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private SwerveSubsystem swerveSubsystem;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier shooterOverride;
    private PIDController rotationPID = new PIDController(2,0,0);

    public TeleopSwerve(SwerveSubsystem swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier shooterOverride) {
        this.swerveSubsystem = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.shooterOverride = shooterOverride;
    }

    @Override
    public void execute() {
        Pose2d pose = swerveSubsystem.getPose();

        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal;

        /* Exponential Drive */
        translationVal = Math.copySign(Math.pow(translationVal, 2), translationVal);
        strafeVal = Math.copySign(Math.pow(strafeVal, 2), strafeVal);
        
        if (shooterOverride.getAsBoolean()){ // Lock robot angle to shooter
            rotationVal = rotationPID.calculate(swerveSubsystem.getHeading().getRadians(), FiringSolutions.getAngleToSpeaker(pose.getX(), pose.getY()));
        } else {
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
            rotationVal = Math.copySign(Math.pow(rotationVal, 2), rotationVal);
        }

        /* Drive */
        swerveSubsystem.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}