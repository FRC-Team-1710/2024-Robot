package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private final VisionSubsystem vision;
    private final SwerveDrivePoseEstimator swerveOdomEstimator;

    private Field2d m_field = new Field2d();

    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Fused Pose", Pose2d.struct).publish();

    public SwerveSubsystem(VisionSubsystem vision) {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "carnivorous rex");   
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.46, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                checkRedAlliance(),
                this // Reference to this subsystem to set requirements
        );

        // Vision Standard Deviation - Smaller means trust more
        Vector<N3> stateStdDevs = VecBuilder.fill(1, 1, 0.1); // Encoder Odometry
        Vector<N3> visionStdDevs = VecBuilder.fill(1, 1, 2); // Vision Odometry

        swerveOdomEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(1.39, 5.55, new Rotation2d(0)), //TODO change based on auto
                stateStdDevs,
                visionStdDevs);

        this.vision = vision;
    }

    /** Check alliance for the AutoBuilder. Returns true when red. Using a method for better readability */
    public BooleanSupplier checkRedAlliance() {
        return () -> Robot.getAlliance();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double translationX = translation.getX();
        double translationY = translation.getY();

        if (Robot.getAlliance()){
            translationX *= -1;
            translationY *= -1;
        }

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationX,
                        translationY,
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translationX,
                                translationY,
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /** Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speed) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speed, 0.02);

        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return swerveOdomEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swerveOdomEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Pose2d getEstimatedPosition() {
        return swerveOdomEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdomEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        double radians = Robot.getAlliance() ? Math.PI : 0;
        swerveOdomEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d(radians)));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    /**
     * See
     * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
     */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        swerveOdomEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    @Override
    public void periodic() {
        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        // Correct pose estimate with multiple vision measurements
        Optional <EstimatedRobotPose> OptionalEstimatedPoseFront = vision.photonEstimatorFront.update();
        if (OptionalEstimatedPoseFront.isPresent()) {
            final EstimatedRobotPose estimatedPose = OptionalEstimatedPoseFront.get();
            swerveOdomEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        }

       /*  Optional <EstimatedRobotPose> OptionalEstimatedPoseBack = vision.photonEstimatorBack.update();
        if (OptionalEstimatedPoseBack.isPresent()) {
            final EstimatedRobotPose estimatedPose = OptionalEstimatedPoseBack.get();
            swerveOdomEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        }*/

        swerveOdomEstimator.update(getGyroYaw(), getModulePositions());

        m_field.setRobotPose(getEstimatedPosition());

        posePublisher.set(getPose());

        SmartDashboard.putData("field", m_field);
        SmartDashboard.putString("Obodom", getEstimatedPosition().toString());
        SmartDashboard.putNumber("Gyro", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    }
}