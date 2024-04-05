// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.math.FiringSolutionsV3;
import frc.lib.math.Interpolations;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {

    // Devices
    private CANSparkBase m_Wrist;
    private CANSparkBase shootaTop; // leader
    private CANSparkBase shootaBot;

    private RelativeEncoder m_VelocityEncoder;
    private RelativeEncoder m_VelocityEncoder2;
    /** NEO Encoder */
    private RelativeEncoder m_PositionEncoder;
    private CANcoder m_WristEncoder;

    // PID
    private PIDController m_pidWrist;
    private SparkPIDController botPID;
    private SparkPIDController topPID;

    /* PID Constants
    private double velocityP = 0.0006;
    private double velocityI = 7e-7;
    private double velocityD = 0; */

    // Old PID Constants
    private double velocityP = 0.0004;
    private double velocityI = 5.6e-7;
    private double velocityD = 0;

    private double positionP = 1;
    private double positionI = 0;
    private double positionD = 0;

    // Vars
    /** In m/s */
    private double shooterVelocity = 17.10; // real

    private double shooterAngleToSpeaker, shooterAngleToAmp;
    private boolean CanEncoderHasFailed = false;
    private boolean ENCFAIL = false;
    public boolean isZeroed = false;
    public boolean wristIsLocked = false;
    /** IN RADIANS */
    private double angleOffset = Constants.Shooter.angleOffsetTop;

    private double distanceToMovingSpeakerTarget = .94;
    public double lastWristAngleSetpoint = 0.0;
    public boolean manualOverride = false;
    public double wristAngleUpperBound;
    public double wristAngleLowerBound;
    public boolean outsideAllianceWing = false;
    private boolean wristCoast = false;

    private double interpolationOffset = -2.5;

    private Joystick controller;

    // Constants
    private final double wristAngleMax = 0.0;
    private final double wristAngleMin = 0.0;
    private final Timer speedTimer = new Timer();
    private final Interpolations interpolation = new Interpolations();
    private final int m_WristCurrentMax = 84;

    private SwerveSubsystem swerveSubsystem;
    private ElevatorSubsystem elevatorSubsystem;

    public ShooterSubsystem(
            SwerveSubsystem swerve, ElevatorSubsystem elevator, Joystick controller) {
        swerveSubsystem = swerve;
        elevatorSubsystem = elevator;

        this.controller = controller;

        m_Wrist = new CANSparkMax(13, MotorType.kBrushless);
        shootaTop = new CANSparkMax(11, MotorType.kBrushless); // leader
        shootaBot = new CANSparkMax(12, MotorType.kBrushless);

        // Encoders
        m_VelocityEncoder = shootaTop.getEncoder();
        m_VelocityEncoder2 = shootaBot.getEncoder();
        m_PositionEncoder = m_Wrist.getEncoder();
        m_WristEncoder = new CANcoder(54);

        // CANcoderConfiguration configs = new CANcoderConfiguration();
        // m_WristEncoder.getConfigurator().apply(configs);

        // Spark Max Setup
        shootaTop.restoreFactoryDefaults();
        shootaBot.restoreFactoryDefaults();
        m_Wrist.restoreFactoryDefaults();

        m_Wrist.setIdleMode(IdleMode.kBrake);
        m_Wrist.setInverted(false);

        shootaBot.setInverted(false);

        // PID
        botPID = shootaBot.getPIDController();
        topPID = shootaTop.getPIDController();

        botPID.setP(velocityP, 0);
        botPID.setI(velocityI, 0);
        botPID.setD(velocityD, 0);

        topPID.setP(velocityP, 0);
        topPID.setI(velocityI, 0);
        topPID.setD(velocityD, 0);

        m_VelocityEncoder.setMeasurementPeriod(16);
        m_VelocityEncoder.setAverageDepth(2);

        m_Wrist.burnFlash();
        shootaBot.burnFlash();
        shootaTop.burnFlash();

        m_pidWrist = new PIDController(positionP, positionI, positionD);

        SmartDashboard.putNumber("set velocity", shooterVelocity);

        SmartDashboard.putData("Wrist PID", m_pidWrist);
        SmartDashboard.putData(this);

        SmartDashboard.putNumber("Set Slip Offset", FiringSolutionsV3.slipPercent);
        SmartDashboard.putNumber("Set Target Z", FiringSolutionsV3.speakerTargetZ);

        SmartDashboard.putNumber("Velo P", velocityP);
        SmartDashboard.putNumber("Velo I", velocityI);
        SmartDashboard.putNumber("Velo D", velocityD);

        SmartDashboard.putNumber("Top Bottom Offset", 400);
        SmartDashboard.putBoolean("Wrist Coast", wristCoast);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        tempPIDTuning();

        shooterVelocity = SmartDashboard.getNumber("set velocity", shooterVelocity);
        boolean shooterAtSpeed = isShooterAtSpeed();

        if (shooterAtSpeed) {
            controller.setRumble(RumbleType.kBothRumble, .2);
        } else {
            controller.setRumble(RumbleType.kBothRumble, 0);
        }

        FiringSolutionsV3.slipPercent = SmartDashboard.getNumber("Set Slip Offset", FiringSolutionsV3.slipPercent);
        FiringSolutionsV3.speakerTargetZ = SmartDashboard.getNumber("Set Target Z", FiringSolutionsV3.speakerTargetZ);

        SmartDashboard.putNumber(
                "Top - Bottom error",
                m_VelocityEncoder.getVelocity() - m_VelocityEncoder2.getVelocity());
        SmartDashboard.putNumber("Current Angle Radians", getCurrentShooterAngle());
        SmartDashboard.putNumber("Can Encoder Angle", getCanCoderAngle());
        SmartDashboard.putNumber("Current Velocity", getVelocity());
        SmartDashboard.putBoolean("shooter at speed", shooterAtSpeed);
        SmartDashboard.putBoolean("Can Encoder Has Failed", CanEncoderHasFailed);
        SmartDashboard.putNumber(
                "Current Angle Degrees", Units.radiansToDegrees(getCurrentShooterAngle()));

        // check for encoder failure
        /*
         * if (m_WristEncoder.isConnected()) {
         * ENCFAIL = false;
         * } else {
         * isZeroed = false;
         * revEncoderHasFailed = true;
         * ENCFAIL = true;
         * }
         */

        if (m_WristEncoder.getFault_Hardware().getValue() || m_WristEncoder.getFault_BadMagnet().getValue()){
            ENCFAIL = true;
            CanEncoderHasFailed = true;
            isZeroed = false;
        } else {
            ENCFAIL = false;
        }

        SmartDashboard.putBoolean("ODER FAILURE", ENCFAIL);
        SmartDashboard.putBoolean("Is Wrist Zeroed", isZeroed);

        updateShooterMath();

        SmartDashboard.putNumber("Flywheel Left Current", shootaTop.getOutputCurrent());
        SmartDashboard.putNumber("Flywheel Right Current", shootaBot.getOutputCurrent());
        SmartDashboard.putNumber("Wrist Current", m_Wrist.getOutputCurrent());
        SmartDashboard.putBoolean("is Wrist Stalled", isWristMotorStalled());
        SmartDashboard.putNumber("Wrist Built-in Encoder", getMotorEncoderAngle());

        if (isZeroed) {
            if (!manualOverride) {
                m_Wrist.set(m_pidWrist.calculate(getCurrentShooterAngle(), lastWristAngleSetpoint));
            }

            // Implement whenever build stops throwing
            /*
             * if (getCurrentShooterAngle() > wristAngleUpperBound){
             * m_Wrist.set(m_pidWrist.calculate(getCurrentShooterAngle(),
             * wristAngleUpperBound));
             * } else if (getCurrentShooterAngle() < wristAngleLowerBound){
             * m_Wrist.set(m_pidWrist.calculate(getCurrentShooterAngle(),
             * wristAngleLowerBound));
             * }
             */
        }

        if (SmartDashboard.getBoolean("Wrist Coast", wristCoast) != wristCoast) {
            wristCoast = SmartDashboard.getBoolean("Wrist Coast", wristCoast);
            if (wristCoast){
                setWristToCoast();
            } else {
                setWristToBrake();
            }
        }
    }

    private void tempPIDTuning() {
        if (velocityP != SmartDashboard.getNumber("Velo P", velocityP)){
            velocityP = SmartDashboard.getNumber("Velo P", velocityP);
            topPID.setP(velocityP, 0);
            botPID.setP(velocityP, 0);
        }

        if (velocityI != SmartDashboard.getNumber("Velo I", velocityI)){
            velocityI = SmartDashboard.getNumber("Velo I", velocityI);
            topPID.setI(velocityI, 0);
            botPID.setI(velocityI, 0);
        }

        if (velocityD != SmartDashboard.getNumber("Velo D", velocityD)){
            velocityD = SmartDashboard.getNumber("Velo D", velocityD);
            topPID.setD(velocityD, 0);
            botPID.setD(velocityD, 0);
        }
    }

    /** in RADIANs units MATTER */
    public double getCurrentShooterAngle() {
        if (!ENCFAIL && !CanEncoderHasFailed) {
            return getCanCoderAngle() + angleOffset;
        } else {
            return getMotorEncoderAngle() + angleOffset;
        }
    }

    public double getCanCoderAngle() {
        return ((m_WristEncoder.getPosition().getValueAsDouble() * 2 * Math.PI) / 4);
    }

    public double getMotorEncoderAngle() {
        return ((m_PositionEncoder.getPosition() * 2 * Math.PI) / 100);
    }

    public double getCalculatedAngleToAmp() {
        return shooterAngleToAmp;
    }

    public double getCalculatedAngleToSpeaker() {
        return shooterAngleToSpeaker;
    }

    public double getCalculatedVelocity() {
        return shooterVelocity;
    }

    public double getDistanceTo(double x, double y) {
        return FiringSolutionsV3.getDistanceToTarget(
                swerveSubsystem.getPose().getX(), swerveSubsystem.getPose().getY(), x, y);
    }

    public double getDistanceToSpeakerWhileMoving() {
        return distanceToMovingSpeakerTarget;
    }

    /** Shooter velocity in RPM */
    public double getVelocity() {
        return m_VelocityEncoder.getVelocity();
    }

    public double getWristRotations() {
        return m_PositionEncoder.getPosition();
    }

    /** Get whether shooter is at target speed */
    public boolean isShooterAtSpeed() { // Copied from Hudson but made it better
        // if error less than certain amount start the timer
        if (Math.abs(getVelocity() - FiringSolutionsV3.convertToRPM(shooterVelocity)) < 150) {// TODO Adjust
            speedTimer.start();
        } else {
            speedTimer.reset();
            return false;
        }

        if (speedTimer.get() > .25) {
            return true;
        } else {
            return false;
        }
    }

    /** Check if wrist motor is exceeding stall current, used for zeroing */
    public boolean isWristMotorStalled() {
        return m_Wrist.getOutputCurrent() > m_WristCurrentMax;
    }

    public void PointShoot(double PointAngle, double launchVelocity) {
        setWristByAngle(PointAngle);
        botPID.setReference(launchVelocity, CANSparkMax.ControlType.kVelocity);
        topPID.setReference(launchVelocity, CANSparkMax.ControlType.kVelocity);
    }

    /** The proper wrist encoder reset method. USE ONLY THIS ONE */
    public void resetWristEncoders(double newOffset) {
        angleOffset = newOffset;
        m_WristEncoder.setPosition(0);
        CanEncoderHasFailed = false;
        m_PositionEncoder.setPosition(0);
        isZeroed = true;
        lastWristAngleSetpoint = newOffset;
    }

    public void useBuiltInEncoder(boolean enable) {
        CanEncoderHasFailed = enable;
    }

    /** Wrist Encoder Reset DO NOT USE */
    public void restartWristEncoders() {
        m_WristEncoder.setPosition(0);
        m_PositionEncoder.setPosition(0);
    }

    public void setManualOverride(boolean value) {
        manualOverride = value;
    }

    /** In rotations per minute */
    public void setOffsetVelocity(double velocity) {
        if (velocity == 0) {
            shootaBot.stopMotor();
            shootaTop.stopMotor();
        } else {
            botPID.setReference(
                    velocity - SmartDashboard.getNumber("Top Bottom Offset", 400),
                    CANSparkMax.ControlType.kVelocity);
            topPID.setReference(
                    velocity + SmartDashboard.getNumber("Top Bottom Offset", 400),
                    CANSparkMax.ControlType.kVelocity);
        }
    }

    /** In rotations per minute */
    public void setShooterVelocity(double velocity) {
        if (velocity == 0) {
            shootaBot.stopMotor();
            shootaTop.stopMotor();
        } else {
            botPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
            topPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
        }
    }

    /** Reset wrist encoder to given value DO NOT USE */
    public void setWristEncoderOffset(double newPosition) {
        m_WristEncoder.setPosition(newPosition);
    }

    public void setWristAngleLowerBound(double wristAngleLowerBound) {
        if (wristAngleLowerBound < wristAngleMin) {
            wristAngleLowerBound = wristAngleMin;
        } else if (wristAngleLowerBound > wristAngleUpperBound) {
            wristAngleLowerBound = wristAngleUpperBound;
        } else {
            this.wristAngleLowerBound = wristAngleLowerBound;
        }
    }

    public void setWristAngleUpperBound(double wristAngleUpperBound) {
        if (wristAngleUpperBound > wristAngleMax) {
            wristAngleUpperBound = wristAngleMax;
        } else if (wristAngleUpperBound < wristAngleLowerBound) {
            wristAngleUpperBound = wristAngleLowerBound;
        } else {
            this.wristAngleUpperBound = wristAngleUpperBound;
        }
    }

    /** IN RADIANS */
    public void setWristByAngle(double angle) {
        manualOverride = false;
        if (isZeroed) {
            // manualOverride = false;
            updateWristAngleSetpoint(angle);
        }
    }

    /** IN ROTATIONS */
    public void setWristByRotations(double newPosition) {
        if (isZeroed) {
            m_Wrist.set(m_pidWrist.calculate(getWristRotations(), newPosition));
        }
    }

    public void setWristSpeedManual(double speed) {
        manualOverride = true;
        m_Wrist.set(speed);
    }

    public void setWristToBrake() {
        m_Wrist.setIdleMode(IdleMode.kBrake);
    }

    public void setWristToCoast() {
        m_Wrist.setIdleMode(IdleMode.kCoast);
    }

    public void updateWristAngleSetpoint(double angle) {
        if (angle != lastWristAngleSetpoint) {
            lastWristAngleSetpoint = angle;
        }
    }

    public void offsetUP() {
        interpolationOffset += 0.5;
    }

    public void offsetDOWN() {
        interpolationOffset -= 0.5;
    }

    public void updateShooterMath() { // Shooter Math

        Pose2d pose = swerveSubsystem.getPose();
        ChassisSpeeds chassisSpeeds = swerveSubsystem.getChassisSpeeds();

        distanceToMovingSpeakerTarget = FiringSolutionsV3.getDistanceToMovingTarget(
                pose.getX(),
                pose.getY(),
                FiringSolutionsV3.speakerTargetX,
                FiringSolutionsV3.speakerTargetY,
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                pose.getRotation().getRadians());

        FiringSolutionsV3.updateSpeakerR(distanceToMovingSpeakerTarget);

        // shooterAngleToSpeaker = FiringSolutionsV3.getShooterAngleFromSpeakerR();

        if (elevatorSubsystem.getHeight() > 0.3) {
            shooterAngleToSpeaker = Math.toRadians(interpolation.getShooterAngleFromInterpolationElevatorUp(
                    distanceToMovingSpeakerTarget)
                    + interpolationOffset + 1.0
                    );
        } else {
            shooterAngleToSpeaker = Math.toRadians(
                    interpolation.getShooterAngleFromInterpolation(distanceToMovingSpeakerTarget)
                            + interpolationOffset);
        }

        if ((Robot.getAlliance() && pose.getX() < 16.54 - 5)
                ^ (!Robot.getAlliance() && pose.getX() > 5)) {
            outsideAllianceWing = true;
        } else {
            outsideAllianceWing = false;
        }

        SmartDashboard.putNumber("Interpolation Offset", interpolationOffset);
        SmartDashboard.putNumber(
                "Target Velocity RPM", FiringSolutionsV3.convertToRPM(shooterVelocity));
        SmartDashboard.putNumber("Calculated Angle Radians", shooterAngleToSpeaker);
        SmartDashboard.putNumber("Calculated Angle Degrees", Math.toDegrees(shooterAngleToSpeaker));
        SmartDashboard.putNumber(
                "distance",
                FiringSolutionsV3.getDistanceToTarget(
                        pose.getX(),
                        pose.getY(),
                        FiringSolutionsV3.speakerTargetX,
                        FiringSolutionsV3.speakerTargetY));
        SmartDashboard.putNumber("R", FiringSolutionsV3.getSpeakerR());
        SmartDashboard.putNumber("C", FiringSolutionsV3.C(distanceToMovingSpeakerTarget));
        SmartDashboard.putNumber(
                "quarticA",
                FiringSolutionsV3.quarticA(
                        distanceToMovingSpeakerTarget, FiringSolutionsV3.speakerTargetZ));
        SmartDashboard.putNumber(
                "quarticC",
                FiringSolutionsV3.quarticC(
                        distanceToMovingSpeakerTarget, FiringSolutionsV3.speakerTargetZ));
        SmartDashboard.putNumber(
                "quarticE", FiringSolutionsV3.quarticE(distanceToMovingSpeakerTarget));
        SmartDashboard.putNumber("Distance to Moving Target", distanceToMovingSpeakerTarget);
        SmartDashboard.putNumber("robot vx", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("robot vy", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("dist to speaker", FiringSolutionsV3.getDistanceToTarget(pose.getX(), pose.getY(), FiringSolutionsV3.speakerTargetX, FiringSolutionsV3.speakerTargetY));
        SmartDashboard.putNumber("adjusted target distance", distanceToMovingSpeakerTarget - FiringSolutionsV3.getDistanceToTarget(pose.getX(), pose.getY(), FiringSolutionsV3.speakerTargetX, FiringSolutionsV3.speakerTargetY));

        double distanceToMovingAmpTarget = FiringSolutionsV3.getDistanceToMovingTarget(
                pose.getX(),
                pose.getY(),
                FiringSolutionsV3.ampTargetX,
                FiringSolutionsV3.ampTargetY,
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                pose.getRotation().getRadians());

        FiringSolutionsV3.updateAmpR(distanceToMovingAmpTarget);
        shooterAngleToAmp = FiringSolutionsV3.getShooterAngleFromAmpR();
    }
}
