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
import com.pathplanner.lib.path.PathConstraints;
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
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

public class SwerveSubsystem extends SubsystemBase {

    // Swerve devices
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    // Vars
    private final VisionSubsystem vision;
    private final SwerveDrivePoseEstimator swerveOdomEstimator;

    // Characterization stuff
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> {
                        voltage(volts.in(Units.Volts));
                    },
                    this::sysidroutine,
                    this));

    // Logging
    private Field2d m_field = new Field2d();
    private StructPublisher<Pose2d> posePublisher;
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

        // Auto setup
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
        // TODO tune
        // Vision Standard Deviation - Smaller means trust more
        Vector<N3> stateStdDevs = VecBuilder.fill(1, 1, 1); // Encoder Odometry
        Vector<N3> visionStdDevs = VecBuilder.fill(1.5, 1.5, 10); // Vision Odometry

        // Swerve obodom
        swerveOdomEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(1.35, 5.55, new Rotation2d(0)),
                stateStdDevs,
                visionStdDevs);

        // Logging
        posePublisher = NetworkTableInstance.getDefault().getStructTopic("Fused Pose", Pose2d.struct).publish();
        swervePublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Swerve Module States", SwerveModuleState.struct).publish();

        SmartDashboard.putData(gyro);
        SmartDashboard.putData(this);
        this.vision = vision;
    }

    /** Check alliance for the AutoBuilder. Returns true when red. Using a method for better readability */
    public BooleanSupplier checkRedAlliance() {
        return () -> Robot.getAlliance();
    }

    /** Teleop drive method */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double translationX = translation.getX();
        double translationY = translation.getY();

        if (Robot.getAlliance()) {
            translationX *= -1;
            translationY *= -1;
        }

        SwerveModuleState[] swerveModuleStates;

        if (fieldRelative) {
            swerveModuleStates = Constants.Swerve.swerveKinematics
                    .toSwerveModuleStates(ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
                            translationX,
                            translationY,
                            rotation,
                            getHeading()), 0.02));
        } else {
            swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(translationX,
                    translationY,
                    rotation));
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        swervePublisher.set(swerveModuleStates);
    }

    /** Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /** Drive method for Autos */
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
        swerveOdomEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        double radians = Robot.getAlliance() ? Math.PI : 0;
        swerveOdomEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d(radians)));
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

    public void voltage(double Voltage) {
        for (SwerveModule mod : mSwerveMods) {
            mod.voltageDrive(Voltage);
        }
    }

    @Override
    public void periodic() {
       /*  for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }*/

        // Correct pose estimate with multiple vision measurements
        Optional<EstimatedRobotPose> OptionalEstimatedPoseFront = vision.getEstimatedGlobalPose();
        if (OptionalEstimatedPoseFront.isPresent()) {

            final EstimatedRobotPose estimatedPose = OptionalEstimatedPoseFront.get();

            swerveOdomEstimator
                    .setVisionMeasurementStdDevs(vision.getEstimationStdDevs(estimatedPose.estimatedPose.toPose2d()));

            swerveOdomEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(),
                    estimatedPose.timestampSeconds);
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

    public Command pathfindToSource() { // TODO tune constraints
        PathConstraints constraints = new PathConstraints(4, 3, 10, 10);
        if (Robot.getAlliance()) {
            return AutoBuilder.pathfindToPose(new Pose2d(1.167, 1.508, Rotation2d.fromRadians(-2.099)), constraints, 0);
        } else {
            return AutoBuilder.pathfindToPose(new Pose2d(15.373, 1.419, Rotation2d.fromRadians(-1.044)), constraints, 0);
        }
    }

    public void sysidroutine(SysIdRoutineLog log) {
        log.motor("drive-left")
                .voltage(
                        m_appliedVoltage.mut_replace(
                                mSwerveMods[3].getMotorVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(mSwerveMods[3].getPosition().distanceMeters, Meters))
                .linearVelocity(
                        m_velocity.mut_replace(mSwerveMods[3].getMotorVelocity(), MetersPerSecond));
        log.motor("drive-right")
                .voltage(
                        m_appliedVoltage.mut_replace(
                                mSwerveMods[0].getMotorVoltage() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(mSwerveMods[3].getPosition().distanceMeters, Meters))
                .linearVelocity(
                        m_velocity.mut_replace(mSwerveMods[0].getMotorVelocity(), MetersPerSecond));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}