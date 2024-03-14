package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.math.FiringSolutionsV3;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.IntexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import org.photonvision.targeting.PhotonPipelineResult;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
    private SwerveSubsystem swerveSubsystem;
    private VisionSubsystem vision;
    private ShooterSubsystem shooterSubsystem;
    private IntexerSubsystem intexerSubsystem;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    private BooleanSupplier robotCentricSup;
    private BooleanSupplier shooterOverrideAmp;
    private BooleanSupplier shooterOverrideSpeaker;
    private BooleanSupplier intakeOverride;
    private BooleanSupplier intakeOverrideNoMove;

    private PIDController rotationPID = new PIDController(0.65, 0.00001, 0.04);

    private Joystick controller;

    private boolean noteInside = false;

    public TeleopSwerve(
            SwerveSubsystem swerve,
            VisionSubsystem vision,
            ShooterSubsystem shooter,
            IntexerSubsystem intexer,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            BooleanSupplier shooterOverrideAmp,
            BooleanSupplier shooterOverrideSpeaker,
            BooleanSupplier intake,
            BooleanSupplier intakeNoMove,
            Joystick controller) {

        this.swerveSubsystem = swerve;
        this.vision = vision;
        this.shooterSubsystem = shooter;
        this.intexerSubsystem = intexer;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.shooterOverrideAmp = shooterOverrideAmp;
        this.shooterOverrideSpeaker = shooterOverrideSpeaker;
        this.intakeOverride = intake;
        this.intakeOverrideNoMove = intakeNoMove;
        this.controller = controller;
        SmartDashboard.putData("Lock On Rotation PID", rotationPID);
    }

    @Override
    public void execute() {
        Pose2d pose = swerveSubsystem.getPose();
        boolean robotCentric = robotCentricSup.getAsBoolean();
        boolean openLoop = true;
        PhotonPipelineResult result = vision.getLatestResultN();

        /* Get Values, Deadband */
        double translationVal =
                MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal;

        double offset;
        double ampLockOffset;

        if (Robot.getAlliance()) {
            ampLockOffset = 16.54 - 5;
            if (pose.getRotation().getRadians() > 0) {
                offset = -Math.toRadians(180);
            } else {
                offset = Math.toRadians(180);
            }
        } else {
            ampLockOffset = 5;
            offset = 0;
        }

        /* Exponential Drive */
        translationVal = Math.copySign(Math.pow(translationVal, 2), translationVal);
        strafeVal = Math.copySign(Math.pow(strafeVal, 2), strafeVal);

        if (shooterOverrideSpeaker.getAsBoolean()) { // Lock robot angle to speaker
            if (shooterSubsystem.getDistanceToSpeakerWhileMoving() >= 3.5) {
                controller.setRumble(RumbleType.kBothRumble, 0.5);
            } else {
                controller.setRumble(RumbleType.kBothRumble, 0);
            }

            ChassisSpeeds currentSpeed = swerveSubsystem.getChassisSpeeds();

            rotationVal = rotationPID.calculate(
                    pose.getRotation().getRadians() + offset,
                    FiringSolutionsV3.getAngleToMovingTarget(
                            pose.getX(),
                            pose.getY(),
                            FiringSolutionsV3.speakerTargetX,
                            FiringSolutionsV3.speakerTargetY,
                            currentSpeed.vxMetersPerSecond,
                            currentSpeed.vyMetersPerSecond,
                            pose.getRotation().getRadians()));
            openLoop = false;

        } else if (shooterOverrideAmp.getAsBoolean()) { // Lock robot angle to amp
            ChassisSpeeds currentSpeed = swerveSubsystem.getChassisSpeeds();
            openLoop = false;

            if ((Robot.getAlliance() && pose.getX() < 16.54 - 5)
                    ^ (!Robot.getAlliance() && pose.getX() > 5)) {

                rotationVal = rotationPID.calculate(
                        pose.getRotation().getRadians() + offset,
                        FiringSolutionsV3.getAngleToMovingTarget(
                                pose.getX(),
                                pose.getY(),
                                FiringSolutionsV3.ampTargetX,
                                FiringSolutionsV3.ampTargetY,
                                currentSpeed.vxMetersPerSecond,
                                currentSpeed.vyMetersPerSecond,
                                pose.getRotation().getRadians()));
            } else {
                rotationVal =
                        rotationPID.calculate(pose.getRotation().getRadians(), Math.toRadians(-90));
            }

        } else if (intakeOverride.getAsBoolean()
                && result.hasTargets()
                && !noteInside) { // Lock robot towards detected note

            double yawToNote = Math.toRadians(result.getBestTarget().getYaw())
                    + swerveSubsystem.getGyroYaw().getRadians();

            SmartDashboard.putNumber("Note Yaw", yawToNote);

            rotationVal = rotationPID.calculate(
                    yawToNote, swerveSubsystem.getGyroYaw().getRadians());

            translationVal = 0.35;
            robotCentric = true;
            openLoop = false;

        } else if (intakeOverrideNoMove.getAsBoolean() && result.hasTargets() && !noteInside) {
            double yawToNote = Math.toRadians(result.getBestTarget().getYaw())
                    + swerveSubsystem.getGyroYaw().getRadians();

            SmartDashboard.putNumber("Note Yaw", yawToNote);

            openLoop = false;
            rotationVal = rotationPID.calculate(
                    yawToNote, swerveSubsystem.getGyroYaw().getRadians());

        } else {
            rotationVal =
                    MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
            rotationVal = Math.copySign(Math.pow(rotationVal, 2), rotationVal);
            controller.setRumble(RumbleType.kBothRumble, 0);
            noteInside = false;
        }

        if (Robot.getAlliance() && !robotCentric) { // Invert field oriented for always blue origin
            translationVal = -translationVal;
            strafeVal = -strafeVal;
        }

        if (intexerSubsystem.intakeBreak()) {
            translationVal = 0;
            rotationVal = 0;
            noteInside = true;
        }

        /* Drive */
        swerveSubsystem.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                !robotCentric,
                openLoop);
    }
}
