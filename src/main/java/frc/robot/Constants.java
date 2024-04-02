package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;

import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import java.io.UncheckedIOException;

public final class Constants {
    public static final double stickDeadband = 0.07;

    public static class Vision {
        public static final String kAprilTagCameraFront = "ChristiansFourthEye";
        public static final String kAprilTagCameraBack = "ChristiansThirdEye";
        public static final String kNoteCamera = "OnionRing";

        public static final Transform3d kRobotToCamFront = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(28),
                        Units.inchesToMeters(-1.25),
                        Units.inchesToMeters(14.75)),
                new Rotation3d(0, Units.degreesToRadians(15), 0));

        public static final Transform3d kRobotToCamBack = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(-28), // -7
                        Units.inchesToMeters(11),
                        Units.inchesToMeters(14.5)),
                new Rotation3d(0, Units.degreesToRadians(16.5), Math.PI));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = getFieldLayout();

        // The standard deviations of our vision estimated poses, which affect correction rate
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(4, 4, 20);
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 999);
        public static final Vector<N3> stateStdDevs = VecBuilder.fill(1, 1, .5);

        public static final Pose2d startingPoseBlue = new Pose2d(1.35, 5.55, new Rotation2d(0));
        public static final Pose2d startingPoseRed =
                new Pose2d(15.19, 5.55, new Rotation2d(Math.PI));
    }

    private static AprilTagFieldLayout getFieldLayout() {
        try {
            AprilTagFieldLayout attemptedKTagLayout =
                    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
            return attemptedKTagLayout;
        } catch (UncheckedIOException e) {
            DataLogManager.log(e.getMessage());
            return null;
        }
    }

    public static final class Swerve {
        public static final String canivore = "uno";
        public static final int pigeonID = 13;

        public static final COTSTalonFXSwerveConstants chosenModule =
                COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(
                        COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.75);
        public static final double wheelBase = Units.inchesToMeters(20.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 60;
        public static final int driveCurrentThreshold = 90;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.36;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.072013;
        public static final double driveKV = 2.3106;
        public static final double driveKA = 0.27696;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.25;
        /** Radians per Second */
        public static final double maxAngularVelocity =
                10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /** Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(111.7);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /** Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-79.1);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /** Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 8;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-80.1);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /** Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(111.44);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class Intake {
        public static final double noteOutsideSpeed = 0.65;
        public static final double noteInsideSpeed = 0.25;
        public static final double outakeSpeed = -0.5;
    }

    public static final class Shooter {
        public static final double intakeAngleRadians = 0.733;
        public static final double idleSpeedRPM = 1300; // TODO Adjust
        public static final double angleOffsetBottom = Units.degreesToRadians(-9);
        public static final double angleOffsetTop = Units.degreesToRadians(64.8);
        public static final double angleOffsetAuto = Units.degreesToRadians(75.5);
    }

    public static final class Elevator {
        public static final double maxHeightMeters = 0.61;
        public static final double minHeightMeters = -0.015;
        public static final double ampHeight = 0.555;
        public static final double antiBozoSmileToasterAhhGoonBotShooterHeight = .522;
    }

    public static final class Auto {
        public static final double kMaxSpeedMetersPerSecond = 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 9.424778;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 12.56637;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static final PathConstraints PathfindingConstraints = new PathConstraints(
                kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared,
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);

        /** Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
