// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.FiringSolutionsV3;
import frc.lib.math.Interpolations;
import frc.robot.Constants;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class ShooterSubsystem extends SubsystemBase {

    // Devices
    private CANSparkBase m_Wrist;
    private CANSparkBase shootaTop; // leader
    private CANSparkBase shootaBot;

    private RelativeEncoder m_VelocityEncoder;
    private RelativeEncoder m_VelocityEncoder2;
    private RelativeEncoder m_PositionEncoder;
    private DutyCycleEncoder m_WristEncoder;

    // PID
    private PIDController m_pidWrist;
    private SparkPIDController botPID;
    private SparkPIDController topPID;

    // PID Constants
    private double velocityP = 0.0001;
    private double velocityI = 5.2e-7;
    private double velocityD = 0;

    private double positionP = 1;
    private double positionI = 0;
    private double positionD = 0;

    // Vars
    /** In m/s */
    private double shooterVelocity = 12.1;
    private double shooterAngleToSpeaker, shooterAngleToAmp;
    private boolean ENCFAIL = false;
    public boolean isZeroed = false;
    public boolean wristIsLocked = false;
    /** IN RADIANS */
    private double angleOffset = Constants.Shooter.angleOffsetManual;
    private double distanceToMovingSpeakerTarget = .94;
    public double lastWristAngleSetpoint = 0.0;
    public boolean manualOverride = false;
    public double wristAngleUpperBound;
    public double wristAngleLowerBound;

    public boolean outOfAmpRange = false;

    // Constants
    private final double wristAngleMax = 0.0;
    private final double wristAngleMin = 0.0;
    private final Timer speedTimer = new Timer();
    private final Interpolations interpolation = new Interpolations();
    private final int m_WristCurrentMax = 84;

    private SwerveSubsystem swerveSubsystem;

    public ShooterSubsystem(SwerveSubsystem swerve) {
        swerveSubsystem = swerve;

        m_Wrist = new CANSparkMax(13, MotorType.kBrushless);
        shootaTop = new CANSparkMax(11, MotorType.kBrushless); // leader
        shootaBot = new CANSparkMax(12, MotorType.kBrushless);

        // Encoders
        m_VelocityEncoder = shootaTop.getEncoder();
        m_VelocityEncoder2 = shootaBot.getEncoder();
        m_PositionEncoder = m_Wrist.getEncoder();
        m_WristEncoder = new DutyCycleEncoder(0);

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

        m_Wrist.burnFlash();
        shootaBot.burnFlash();
        shootaTop.burnFlash();

        m_pidWrist = new PIDController(positionP, positionI, positionD);

        m_VelocityEncoder.setMeasurementPeriod(20);

        SmartDashboard.putNumber("set velocity", shooterVelocity);

        SmartDashboard.putData(m_pidWrist);
        SmartDashboard.putData(this);

        SmartDashboard.putNumber("Set Slip Offset", FiringSolutionsV3.slipPercent);
        SmartDashboard.putNumber("Set Target Z", FiringSolutionsV3.speakerTargetZ);

        SmartDashboard.putNumber("Velo P", velocityP);
        SmartDashboard.putNumber("Velo I", velocityI);
        SmartDashboard.putNumber("Velo D", velocityD);

        SmartDashboard.putNumber("Top Bottom Offset", 400);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /* 
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
        */
        shooterVelocity = SmartDashboard.getNumber("set velocity", shooterVelocity);

        FiringSolutionsV3.slipPercent = SmartDashboard.getNumber("Set Slip Offset", FiringSolutionsV3.slipPercent);
        FiringSolutionsV3.speakerTargetZ = SmartDashboard.getNumber("Set Target Z", FiringSolutionsV3.speakerTargetZ);

        SmartDashboard.putNumber("Top - Bottom error",
                m_VelocityEncoder.getVelocity() - m_VelocityEncoder2.getVelocity());
        SmartDashboard.putNumber("Current Angle Radians", getCurrentShooterAngle());
        SmartDashboard.putNumber("Current Velocity", getVelocity());
        SmartDashboard.putBoolean("shooter at speed", isShooterAtSpeed());
        SmartDashboard.putNumber("Current Angle Degrees", Units.radiansToDegrees(getCurrentShooterAngle()));

        // check for encoder failure
        if (m_WristEncoder.isConnected()) {
            ENCFAIL = false;
        } else {
            ENCFAIL = true;
        }
        SmartDashboard.putBoolean("ODER FAILURE", ENCFAIL);

        updateShooterMath();

        SmartDashboard.putNumber("Flywheel Left Current", shootaTop.getOutputCurrent());
        SmartDashboard.putNumber("Flywheel Right Current", shootaBot.getOutputCurrent());
        SmartDashboard.putNumber("Wrist Current", m_Wrist.getOutputCurrent());
        SmartDashboard.putBoolean("is Wrist Stalled", isWristMotorStalled());

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

    }

    /** in RADIANs units MATTER */
    public double getCurrentShooterAngle() {
        if (!ENCFAIL) {
            return ((m_WristEncoder.get() * 2 * Math.PI) / 4) + angleOffset;
        } else {
            return ((m_PositionEncoder.getPosition() * 2 * Math.PI) / 100);
        }
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
        return FiringSolutionsV3.getDistanceToTarget(swerveSubsystem.getPose().getX(), swerveSubsystem.getPose().getY(),
                x, y);
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
        if (Math.abs(getVelocity() - FiringSolutionsV3.convertToRPM(shooterVelocity)) < 80) {
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
        if (m_Wrist.getOutputCurrent() > m_WristCurrentMax) {
            return true;
        } else {
            return false;
        }
    }

    public void PointShoot(double PointAngle, double launchVelocity) {
        setWristByAngle(PointAngle);
        botPID.setReference(launchVelocity, CANSparkMax.ControlType.kVelocity);
        topPID.setReference(launchVelocity, CANSparkMax.ControlType.kVelocity);
    }

    public void resetWristEncoders(double newOffset) {
        angleOffset = newOffset;
        m_WristEncoder.reset();
        m_PositionEncoder.setPosition(0);
        isZeroed = true;
    }

    /** Wrist Encoder Reset */
    public void restartWristEncoders(){
        m_WristEncoder.reset();
        m_PositionEncoder.setPosition(0);
    }

    public void setManualWristSpeed(double speed) {
        manualOverride = true;
        m_Wrist.set(speed);
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
            botPID.setReference(velocity - SmartDashboard.getNumber("Top Bottom Offset", 400),
                    CANSparkMax.ControlType.kVelocity);
            topPID.setReference(velocity + SmartDashboard.getNumber("Top Bottom Offset", 400),
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

    /** Reset wrist encoder to given value */
    public void setWristEncoderOffset(double newPosition) {
        m_WristEncoder.setPositionOffset(newPosition);
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
            //manualOverride = false;
            updateWristAngleSetpoint(angle);
        }
    }

    /** IN ROTATIONS */
    public void setWristByRotations(double newPosition) {
        if (isZeroed) {
            m_Wrist.set(m_pidWrist.calculate(getWristRotations(), newPosition));
        }
    }

    public void updateWristAngleSetpoint(double angle) {
        if (angle != lastWristAngleSetpoint) {
            lastWristAngleSetpoint = angle;
        }
    }

    public void updateShooterMath() { // Shooter Math

        Pose2d pose = swerveSubsystem.getPose();
        ChassisSpeeds chassisSpeeds = swerveSubsystem.getChassisSpeeds();

        distanceToMovingSpeakerTarget = FiringSolutionsV3.getDistanceToMovingTarget(pose.getX(), pose.getY(),
                FiringSolutionsV3.speakerTargetX, FiringSolutionsV3.speakerTargetY,
                chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, pose.getRotation().getRadians());

        FiringSolutionsV3.updateSpeakerR(distanceToMovingSpeakerTarget);

        // shooterAngleToSpeaker = FiringSolutionsV3.getShooterAngleFromSpeakerR();
        shooterAngleToSpeaker = Math.toRadians(interpolation.getShooterAngleFromInterpolation(distanceToMovingSpeakerTarget));

        if (FiringSolutionsV3.getDistanceToMovingTarget(pose.getX(), pose.getY(),
                FiringSolutionsV3.ampTargetX, FiringSolutionsV3.ampTargetY,
                chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
                pose.getRotation().getRadians()) > FiringSolutionsV3.maxRangeWithR){
                    outOfAmpRange = true;
                } else {
                    if (outOfAmpRange){
                        outOfAmpRange = false;
                        FiringSolutionsV3.resetAmpR();
                    }
                }

        SmartDashboard.putNumber("Target Velocity RPM", FiringSolutionsV3.convertToRPM(shooterVelocity));
        SmartDashboard.putNumber("Calculated Angle Radians", shooterAngleToSpeaker);
        SmartDashboard.putNumber("Calculated Angle Degrees", Math.toDegrees(shooterAngleToSpeaker));
        SmartDashboard.putNumber("distance", FiringSolutionsV3.getDistanceToTarget(pose.getX(), pose.getY(),
                FiringSolutionsV3.speakerTargetX, FiringSolutionsV3.speakerTargetY));
        SmartDashboard.putNumber("R", FiringSolutionsV3.getSpeakerR());
        SmartDashboard.putNumber("C", FiringSolutionsV3.C(distanceToMovingSpeakerTarget));
        SmartDashboard.putNumber("quarticA",
                FiringSolutionsV3.quarticA(distanceToMovingSpeakerTarget, FiringSolutionsV3.speakerTargetZ));
        SmartDashboard.putNumber("quarticC",
                FiringSolutionsV3.quarticC(distanceToMovingSpeakerTarget, FiringSolutionsV3.speakerTargetZ));
        SmartDashboard.putNumber("quarticE", FiringSolutionsV3.quarticE(distanceToMovingSpeakerTarget));
        SmartDashboard.putNumber("Distance to Moving Target", distanceToMovingSpeakerTarget);
        SmartDashboard.putNumber("robot vx", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("robot vy", chassisSpeeds.vyMetersPerSecond);

        double distanceToMovingAmpTarget = FiringSolutionsV3.getDistanceToMovingTarget(pose.getX(), pose.getY(),
                FiringSolutionsV3.ampTargetX, FiringSolutionsV3.ampTargetY,
                chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, pose.getRotation().getRadians());

        FiringSolutionsV3.updateAmpR(distanceToMovingAmpTarget);

        shooterAngleToAmp = FiringSolutionsV3.getShooterAngleFromAmpR();
    }
}
