// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.FiringSolutions;
import frc.lib.math.FiringSolutionsV3;
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
    private CANSparkBase m_Wrist = new CANSparkMax(13, MotorType.kBrushless);
    private CANSparkBase shootaBot = new CANSparkMax(11, MotorType.kBrushless); // leader
    private CANSparkBase shootaTop = new CANSparkMax(12, MotorType.kBrushless);

    private RelativeEncoder m_VelocityEncoder;
    private RelativeEncoder m_PositionEncoder;
    private DutyCycleEncoder m_WristEncoder;

    // PID
    private PIDController m_pidWrist;
    private SparkPIDController leftPID;
    private SparkPIDController rightPID;

    // PID Constants
    private double pidSpdP = 0.00018;
    private double pidSpdI = 5.2e-7;
    private double pidSpdD = 1.5;

    private double pidPosP = 2;
    private double pidPosI = 0;
    private double pidPosD = 0;

    // Vars
    private double shooterVelocity = 12;
    private double shooterAngle;
    private Boolean ENCFAIL = false;
    public boolean isZeroed = false;
    private double angleOffset = 68.2; // IN RADIANS
    private final Timer speedTimer = new Timer();
    private final int m_WristCurrentMax = 84;

    private SwerveSubsystem swerveSubsystem;

    public ShooterSubsystem(SwerveSubsystem swerve) {
        swerveSubsystem = swerve;

        // Encoders
        m_VelocityEncoder = shootaBot.getEncoder();
        m_PositionEncoder = m_Wrist.getEncoder();
        m_WristEncoder = new DutyCycleEncoder(0);

        // Spark Max Setup
        shootaBot.restoreFactoryDefaults();
        shootaTop.restoreFactoryDefaults();
        m_Wrist.restoreFactoryDefaults();
        m_Wrist.setIdleMode(IdleMode.kBrake);
        m_Wrist.setInverted(true);
        m_Wrist.burnFlash();
        shootaTop.setInverted(false);
        shootaTop.burnFlash();

        // PID
        leftPID = shootaBot.getPIDController();
        rightPID = shootaTop.getPIDController();
        m_pidWrist = new PIDController(pidPosP, pidPosI, pidPosD);

        SmartDashboard.putNumber("set velocity", shooterVelocity);

        SmartDashboard.putData(m_pidWrist);
        SmartDashboard.putData(this);

        SmartDashboard.putNumber("Set Slip Offset", FiringSolutionsV3.slipPercent);
        SmartDashboard.putNumber("Set Target Z", FiringSolutionsV3.shooterTargetZ);

        SmartDashboard.putNumber("Velo P", pidSpdP);
        SmartDashboard.putNumber("Velo I", pidSpdI);
        SmartDashboard.putNumber("Velo D", pidSpdD);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // TODO Remove when done tuning
        pidSpdP = SmartDashboard.getNumber("Velo P", pidSpdP);
        pidSpdI = SmartDashboard.getNumber("Velo I", pidSpdI);
        pidSpdD = SmartDashboard.getNumber("Velo D", pidSpdD);

        leftPID.setP(pidSpdP, 0);
        leftPID.setI(pidSpdI, 0);
        leftPID.setD(pidSpdD, 0);

        rightPID.setP(pidSpdP, 0);
        rightPID.setI(pidSpdI, 0);
        rightPID.setD(pidSpdD, 0);

        //setpointv = SmartDashboard.getNumber("set velocity", setpointv);
        shooterVelocity = SmartDashboard.getNumber("set velocity", shooterVelocity);

        FiringSolutionsV3.slipPercent = SmartDashboard.getNumber("Set Slip Offset", FiringSolutionsV3.slipPercent);
        FiringSolutionsV3.shooterTargetZ = SmartDashboard.getNumber("Set Target Z", FiringSolutionsV3.shooterTargetZ);

        SmartDashboard.putNumber("Current Angle Radians", getAngle());
        SmartDashboard.putNumber("Current Velocity", getVelocity());
        SmartDashboard.putBoolean("shooter at speed", shooterAtSpeed());
        SmartDashboard.putNumber("Current Angle Degrees", Units.radiansToDegrees(getAngle()));

        // check for encoder failure
        if (m_WristEncoder.isConnected()) {
            ENCFAIL = false;
        } else {
            ENCFAIL = true;
        }
        SmartDashboard.putBoolean("ODER FAILURE", ENCFAIL);
        updateShooterMath();

        SmartDashboard.putNumber("Flywheel Left Current", shootaBot.getOutputCurrent());
        SmartDashboard.putNumber("Flywheel Right Current", shootaTop.getOutputCurrent());
        SmartDashboard.putNumber("Wrist Current", m_Wrist.getOutputCurrent());
        SmartDashboard.putBoolean("is Wrist Stalled", isWristMotorStalled());
    }

    /** Check if wrist motor is exceeding stall current, used for zeroing */
    public boolean isWristMotorStalled() {
        if (m_Wrist.getOutputCurrent() > m_WristCurrentMax) {
            return true;
        } else {
            return false;
        }
    }

    /** Reset wrist encoder to given value */
    public void setWristEncoderPosition(double newPosition) {
        m_WristEncoder.setPositionOffset(newPosition);
    }

    /** Get whether shooter is at target speed */
    public boolean shooterAtSpeed() { // Copied from Hudson but made it better

        // if error less than certain amount start the timer
        if (Math.abs(getVelocity() - FiringSolutionsV3.convertToRPM(shooterVelocity)) < 50) {
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

    /** Shooter velocity in RPM */
    public double getVelocity() {
        return m_VelocityEncoder.getVelocity();
    }

    /** in RADIANs units MATTER */
    public double getAngle() {
        if (!ENCFAIL) {
            return ((-m_WristEncoder.get() * 2 * Math.PI) / 4) + angleOffset;
        } else {
            return ((m_PositionEncoder.getPosition() * 2 * Math.PI) / 100) + angleOffset;
        }
    }

    public void resetWristEncoder(double newOffset) {
        angleOffset = newOffset;
        m_WristEncoder.reset();
        m_PositionEncoder.setPosition(0);
        isZeroed = true;
    }

    /** IN RADIANS */
    public void setWristPosition(double angle) {
        m_Wrist.set(m_pidWrist.calculate(getAngle(), angle));
    }

    /** In rotations per minute */
    public void SetShooterVelocity(double velocity) {
        if (velocity == 0) {
            shootaTop.stopMotor();
            shootaBot.stopMotor();
        } else {
            leftPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
            rightPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
        }
    }

    /** In rotations per minute */
    public void SetOffsetVelocity(double velocity) {
        if (velocity == 0) {
            shootaTop.stopMotor();
            shootaBot.stopMotor();
        } else {
            leftPID.setReference(velocity - 200, CANSparkMax.ControlType.kVelocity);
            rightPID.setReference(velocity + 200, CANSparkMax.ControlType.kVelocity);
        }
    }

    public void PointShoot(double PointAngle, double launchVelocity) {
        m_Wrist.set(m_pidWrist.calculate(getAngle(), PointAngle));
        leftPID.setReference(launchVelocity, CANSparkMax.ControlType.kVelocity);
        rightPID.setReference(launchVelocity, CANSparkMax.ControlType.kVelocity);
    }

    public void manualWristSpeed(double speed) {
        m_Wrist.set(speed);
    }

    public double getCalculatedVelocity() {
        return shooterVelocity;
    }

    public double getCalculatedAngle() {
        return shooterAngle;
    }

    public void updateShooterMath() { // Shooter Math
        Pose2d pose = swerveSubsystem.getPose();
        ChassisSpeeds chassisSpeeds = swerveSubsystem.getChassisSpeeds();

        double angleToSpeaker = FiringSolutionsV3.getAngleToSpeaker(pose.getX(), pose.getY());

        double angleToMovingTarget = FiringSolutionsV3.getAngleToMovingTarget(pose.getX(), pose.getY(),
                chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, pose.getRotation().getRadians());

        double robotVelocityTowardsSpeaker = FiringSolutionsV3.getRobotVelocityTowardsSpeaker(
                chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
                angleToSpeaker,
                pose.getRotation().getRadians());

        double robotVelocityPerpendicularToSpeaker = FiringSolutionsV3.getRobotVelocityPerpendicularToSpeaker(
                chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
                angleToSpeaker,
                pose.getRotation().getRadians());

        double distanceToMovingTarget = FiringSolutionsV3.getDistanceToMovingTarget(pose.getX(), pose.getY(),
                chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, pose.getRotation().getRadians());

        FiringSolutionsV3.updateR(distanceToMovingTarget);

        shooterAngle = FiringSolutionsV3.getShooterAngle();

        SmartDashboard.putNumber("Target Velocity RPM", FiringSolutions.convertToRPM(shooterVelocity));
        SmartDashboard.putNumber("Calculated Angle Radians", shooterAngle);
        SmartDashboard.putNumber("Calculated Angle Degrees", Math.toDegrees(shooterAngle));
        SmartDashboard.putNumber("distance", FiringSolutionsV3.getDistanceToSpeaker(pose.getX(), pose.getY()));
        SmartDashboard.putNumber("angle to speaker", angleToSpeaker);
        SmartDashboard.putNumber("R", FiringSolutionsV3.getR());
        SmartDashboard.putNumber("C", FiringSolutionsV3.C(distanceToMovingTarget));
        SmartDashboard.putNumber("quarticA", FiringSolutionsV3.quarticA(distanceToMovingTarget));
        SmartDashboard.putNumber("quarticC", FiringSolutionsV3.quarticC(distanceToMovingTarget));
        SmartDashboard.putNumber("quarticE", FiringSolutionsV3.quarticE(distanceToMovingTarget));
        SmartDashboard.putNumber("Angle to Moving Target", angleToMovingTarget);
        SmartDashboard.putNumber("Distance to Moving Target", distanceToMovingTarget);
        SmartDashboard.putNumber("Robot Velocity Towards Speaker", robotVelocityTowardsSpeaker);
        SmartDashboard.putNumber("Robot Velocity Perpendicular to Speaker", robotVelocityPerpendicularToSpeaker);
        SmartDashboard.putNumber("target x", FiringSolutionsV3.movingTarget(pose.getX(), pose.getY(),
                robotVelocityTowardsSpeaker, robotVelocityPerpendicularToSpeaker).get().getX());
        SmartDashboard.putNumber("target y",
                FiringSolutionsV3.movingTarget(pose.getX(), pose.getY(), robotVelocityTowardsSpeaker, robotVelocityPerpendicularToSpeaker).get().getY());
        SmartDashboard.putNumber("robot vx", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("robot vy", chassisSpeeds.vyMetersPerSecond);
    }
}