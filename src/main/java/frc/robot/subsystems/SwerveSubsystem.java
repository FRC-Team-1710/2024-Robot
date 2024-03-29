package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.lib.math.FiringSolutionsV3;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SwerveModule;

import org.photonvision.EstimatedRobotPose;

import java.util.Optional;
import java.util.function.BooleanSupplier;

public class SwerveSubsystem extends SubsystemBase {

    // Swerve devices
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    // Vars
    private final VisionSubsystem vision;
    private final SwerveDrivePoseEstimator swerveOdomEstimator;
    private final SwerveDriveOdometry encoderOdometry;
    private SwerveModuleState[] swerveModuleStates;
    private SwerveModulePosition[] swerveModulePositions;

    // Characterization stuff
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> {
                        voltageDrive(volts.in(Units.Volts));
                    },
                    this::sysidroutine,
                    this));

    // Logging
    private Field2d m_field = new Field2d();
    private StructPublisher<Pose2d> posePublisher;
    private StructPublisher<Pose2d> posePublisher2;
    private StructArrayPublisher<SwerveModuleState> swervePublisher;

    // Constructor
    public SwerveSubsystem(VisionSubsystem vision) {
        // Gyro setup
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.canivore);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        // Swerve setup
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };

        // Auto setup spotless:off
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5, 0, 0), // Translation PID constants
                        new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.37268, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(true, true, .5, .25)),
                checkRedAlliance(),
                this // Reference to this subsystem to set requirements
                ); // spotless:on

        swerveModulePositions = getModulePositions();

        // Swerve obodom
        swerveOdomEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getGyroYaw(),
                swerveModulePositions,
                Robot.getAlliance()
                        ? Constants.Vision.startingPoseRed
                        : Constants.Vision.startingPoseBlue,
                Constants.Vision.stateStdDevs,
                Constants.Vision.kSingleTagStdDevs);

        encoderOdometry = new SwerveDriveOdometry(
                Constants.Swerve.swerveKinematics, getGyroYaw(), swerveModulePositions);

        // Logging
        posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Fused Pose", Pose2d.struct)
                .publish();
        posePublisher2 = NetworkTableInstance.getDefault()
                .getStructTopic("Encoder Pose", Pose2d.struct)
                .publish();
        swervePublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Swerve Module States", SwerveModuleState.struct)
                .publish();

        SmartDashboard.putData("Gyro", gyro);
        SmartDashboard.putData(this);
        SmartDashboard.putData("field", m_field);

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            m_field.getObject("field").setPoses(poses);
        });

        this.vision = vision;
    }

    /** Check alliance for the AutoBuilder. Returns true when red. Using a method for better readability */
    public BooleanSupplier checkRedAlliance() {
        return () -> Robot.getAlliance();
    }

    /** Teleop drive method */
    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double translationX = translation.getX();
        double translationY = translation.getY();
        SwerveModuleState[] desiredStates;

        if (fieldRelative) {
            desiredStates =
                    Constants.Swerve.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translationX, translationY, rotation, getHeading()),
                            0.02));
        } else {
            desiredStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                    new ChassisSpeeds(translationX, translationY, rotation));
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /** Used by setChassisSpeeds() in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        return swerveModuleStates;
    }

    public void updateModuleStates() {
        for (SwerveModule mod : mSwerveMods) {
            swerveModuleStates[mod.moduleNumber] = mod.getState();
        }
    }

    /** Drive method for Autos */
    public void setChassisSpeeds(ChassisSpeeds speed) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speed, 0.02);

        SwerveModuleState[] desiredState =
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(desiredState);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdomEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swerveOdomEstimator.resetPosition(getGyroYaw(), swerveModulePositions, pose);
        encoderOdometry.resetPosition(getGyroYaw(), swerveModulePositions, pose);
    }

    public void setPoseToPodium() {
        double xPos = Robot.getAlliance() ? (16.54 - (2.95 - .245)) : (2.95 - .245);
        swerveOdomEstimator.resetPosition(
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(xPos, 4.1, new Rotation2d(getPose().getRotation().getRadians())));
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdomEstimator.resetPosition(
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdomEstimator.resetPosition(
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(
                        getPose().getTranslation(),
                        new Rotation2d(Robot.getAlliance() ? Math.PI : 0)));
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
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        swerveOdomEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    public void voltageDrive(double Voltage) {
        for (SwerveModule mod : mSwerveMods) {
            mod.voltageDrive(Voltage);
        }
    }

    @Override
    public void periodic() {
        updateModuleStates();
        swerveModulePositions = getModulePositions();
        Rotation2d gyroYaw = getGyroYaw();

        // Correct pose estimate with multiple vision measurements
        Optional<EstimatedRobotPose> OptionalEstimatedPoseFront = vision.getEstimatedPoseFront();
        if (OptionalEstimatedPoseFront.isPresent()) {

            final EstimatedRobotPose estimatedPose = OptionalEstimatedPoseFront.get();

            swerveOdomEstimator.setVisionMeasurementStdDevs(
                    vision.getEstimationStdDevsFront(estimatedPose.estimatedPose.toPose2d()));

            swerveOdomEstimator.addVisionMeasurement(
                    estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        }

        Optional<EstimatedRobotPose> OptionalEstimatedPoseBack = vision.getEstimatedPoseBack();
        if (OptionalEstimatedPoseBack.isPresent()) {

            final EstimatedRobotPose estimatedPose2 = OptionalEstimatedPoseBack.get();

            swerveOdomEstimator.setVisionMeasurementStdDevs(
                    vision.getEstimationStdDevsBack(estimatedPose2.estimatedPose.toPose2d()));

            swerveOdomEstimator.addVisionMeasurement(
                    estimatedPose2.estimatedPose.toPose2d(), estimatedPose2.timestampSeconds);
        }

        // Logging
        swerveOdomEstimator.update(gyroYaw, swerveModulePositions);
        encoderOdometry.update(gyroYaw, swerveModulePositions);

        m_field.setRobotPose(getPose());

        posePublisher.set(getPose());
        posePublisher2.set(encoderOdometry.getPoseMeters());
        swervePublisher.set(swerveModuleStates);

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Angle",
                    swerveModulePositions[mod.moduleNumber].angle.getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Velocity",
                    swerveModuleStates[mod.moduleNumber].speedMetersPerSecond);
        }

        // SmartDashboard.putString("Obodom", getPose().toString());
        SmartDashboard.putNumber("Gyro", gyroYaw.getDegrees());
        SmartDashboard.putNumber("Heading", getHeading().getDegrees());
        SmartDashboard.putNumber(
                "Angle to target",
                Math.toDegrees(FiringSolutionsV3.getAngleToMovingTarget(
                        getPose().getX(),
                        getPose().getY(),
                        FiringSolutionsV3.ampTargetX,
                        FiringSolutionsV3.ampTargetY,
                        getChassisSpeeds().vxMetersPerSecond,
                        getChassisSpeeds().vyMetersPerSecond,
                        getPose().getRotation().getRadians())));
    }

    public void sysidroutine(SysIdRoutineLog log) {
        log.motor("drive-BR")
                .voltage(m_appliedVoltage.mut_replace(mSwerveMods[3].getMotorVoltage(), Volts))
                .linearPosition(
                        m_distance.mut_replace(mSwerveMods[3].getPosition().distanceMeters, Meters))
                .linearVelocity(
                        m_velocity.mut_replace(mSwerveMods[3].getMotorVelocity(), MetersPerSecond));

        log.motor("drive-FL")
                .voltage(m_appliedVoltage.mut_replace(mSwerveMods[0].getMotorVoltage(), Volts))
                .linearPosition(
                        m_distance.mut_replace(mSwerveMods[0].getPosition().distanceMeters, Meters))
                .linearVelocity(
                        m_velocity.mut_replace(mSwerveMods[0].getMotorVelocity(), MetersPerSecond));

        log.motor("drive-FR")
                .voltage(m_appliedVoltage.mut_replace(mSwerveMods[1].getMotorVoltage(), Volts))
                .linearPosition(
                        m_distance.mut_replace(mSwerveMods[1].getPosition().distanceMeters, Meters))
                .linearVelocity(
                        m_velocity.mut_replace(mSwerveMods[1].getMotorVelocity(), MetersPerSecond));

        log.motor("drive-BL")
                .voltage(m_appliedVoltage.mut_replace(mSwerveMods[2].getMotorVoltage(), Volts))
                .linearPosition(
                        m_distance.mut_replace(mSwerveMods[2].getPosition().distanceMeters, Meters))
                .linearVelocity(
                        m_velocity.mut_replace(mSwerveMods[2].getMotorVelocity(), MetersPerSecond));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return new SequentialCommandGroup(
                new InstantCommand(this::resetModulesToAbsolute, this),
                new WaitCommand(0.5),
                m_sysIdRoutine.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return new SequentialCommandGroup(
                new InstantCommand(this::resetModulesToAbsolute, this),
                new WaitCommand(0.5),
                m_sysIdRoutine.dynamic(direction));
    }

    // Pathfinding Commands
    public Command pathToSource() {
        if (!Robot.getAlliance()) {
            return AutoBuilder.pathfindToPose(
                    new Pose2d(1.21, 0.96, Rotation2d.fromDegrees(58.79)),
                    Constants.Auto.PathfindingConstraints);
        } else {
            return AutoBuilder.pathfindToPose(
                    new Pose2d(15.47, 1.50, Rotation2d.fromDegrees(-59.53)),
                    Constants.Auto.PathfindingConstraints);
        }
    }

    public Command pathToAmp() {
        if (!Robot.getAlliance()) {
            return AutoBuilder.pathfindToPose(
                    new Pose2d(1.84, 7.59, Rotation2d.fromDegrees(90.37)),
                    Constants.Auto.PathfindingConstraints);
        } else {
            return AutoBuilder.pathfindToPose(
                    new Pose2d(14.74, 7.52, Rotation2d.fromDegrees(90.37)),
                    Constants.Auto.PathfindingConstraints);
        }
    }

    public Command pathToMidfieldChain() {
        if (!Robot.getAlliance()) {
            return AutoBuilder.pathfindToPose(
                    new Pose2d(5.84, 4.1024, Rotation2d.fromDegrees(180)),
                    Constants.Auto.PathfindingConstraints);
        } else {
            return AutoBuilder.pathfindToPoseFlipped(
                    new Pose2d(5.84, 4.1024, Rotation2d.fromDegrees(0)),
                    Constants.Auto.PathfindingConstraints);
        }
    }

    public Command pathToSourceChain() {
        if (!Robot.getAlliance()) {
            return AutoBuilder.pathfindToPose(
                    new Pose2d(4.18, 2.79, Rotation2d.fromDegrees(59.47)),
                    Constants.Auto.PathfindingConstraints);
        } else {
            return AutoBuilder.pathfindToPose(
                    new Pose2d(12.43, 2.90, Rotation2d.fromDegrees(121.26)),
                    Constants.Auto.PathfindingConstraints);
        }
    }

    public Command pathToAmpChain() {
        if (!Robot.getAlliance()) {
            return AutoBuilder.pathfindToPose(
                    new Pose2d(4.20, 5.30, Rotation2d.fromDegrees(-58.21)),
                    Constants.Auto.PathfindingConstraints);
        } else {
            return AutoBuilder.pathfindToPose(
                    new Pose2d(12.50, 5.25, Rotation2d.fromDegrees(-120.96)),
                    Constants.Auto.PathfindingConstraints);
        }
    }
}
