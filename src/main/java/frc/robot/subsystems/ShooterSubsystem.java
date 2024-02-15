// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.FiringSolutions;
import frc.lib.math.FiringSolutionsV2;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShooterSubsystem extends SubsystemBase {

    // Devices
    private CANSparkBase m_Wrist = new CANSparkMax(13, MotorType.kBrushless);
    private CANSparkBase m_ShootaL = new CANSparkMax(11, MotorType.kBrushless); // leader
    private CANSparkBase m_ShootaR = new CANSparkMax(12, MotorType.kBrushless);

    private RelativeEncoder m_VelocityEncoder;
    private RelativeEncoder m_PositionEncoder;
    private DutyCycleEncoder m_WristEncoder;

    // PID
    private PIDController m_pidWrist;
    private SparkPIDController leftPID;
    private SparkPIDController rightPID;

    // PID Constants
    private double pidSpdP = .0000002;
    private double pidSpdI = .0000002;
    private double pidSpdD = .006;

    private double pidPosP = 2;
    private double pidPosI = 0;
    private double pidPosD = 0;

    // Vars
    private final double shooterVelocity = 13;
    private double shooterAngle;
    private double setpointv = 0;
    private double setpointp = 0;
    private Boolean ENCFAIL = false;
    public boolean isZeroed = false;
    private final double angleOffset = -.349; // IN RADIANS

    private SwerveSubsystem swerveSubsystem;

    public ShooterSubsystem(SwerveSubsystem swerve) {
        swerveSubsystem = swerve;

        // Encoders
        m_VelocityEncoder = m_ShootaL.getEncoder();
        m_PositionEncoder = m_Wrist.getEncoder();
        m_WristEncoder = new DutyCycleEncoder(0);

        // Spark Max Setup
        m_ShootaL.restoreFactoryDefaults();
        m_ShootaR.restoreFactoryDefaults();
        m_Wrist.restoreFactoryDefaults();
        m_Wrist.setInverted(true);
        m_Wrist.burnFlash();
        m_ShootaR.setInverted(false);

        // PID
        leftPID = m_ShootaL.getPIDController();
        rightPID = m_ShootaR.getPIDController();
        m_pidWrist = new PIDController(pidPosP, pidPosI, pidPosD);

        SmartDashboard.putNumber("set velocity", 0);
        SmartDashboard.putNumber("set angle", 0);

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

        setpointv = SmartDashboard.getNumber("set velocity", setpointv);
        setpointp = SmartDashboard.getNumber("set angle", setpointp);

        SmartDashboard.putNumber("current angle", getAngle());
        SmartDashboard.putNumber("current velocity", getVelocity());

        // check for encoder failure
        if (m_WristEncoder.isConnected()) {
            ENCFAIL = false;
        } else {
            ENCFAIL = true;
        }
        SmartDashboard.putBoolean("ODER FAILURE", ENCFAIL);
        updateShooterMath();

        //wristManualSet(setpointp);
        //SetShooterVelocity(setpointv);
    }

    public boolean shooterAtSpeed() {
        return (getVelocity() > (shooterVelocity - 50)) && (getVelocity() < (shooterVelocity + 50));
    }

    public double getVelocity() {
        return m_VelocityEncoder.getVelocity();
    }

    public double getAngle() { // in RADIANs units MATTER
        if (!ENCFAIL) {
            return ((-m_WristEncoder.get() * 2 * Math.PI) / 4) + angleOffset;
        } else {
            return ((m_PositionEncoder.getPosition() * 2 * Math.PI) / 100) + angleOffset;
        }
    }

    public void resetWristEncoder() {
        m_WristEncoder.reset();
        m_PositionEncoder.setPosition(0);
        isZeroed = true;
    }

    public void setWristPosition(double angle) {
        m_Wrist.set(m_pidWrist.calculate(getAngle(), angle));
    }

    public void SetShooterVelocity(double velocity) {
        if (velocity == 0) {
            m_ShootaR.stopMotor();
            m_ShootaL.stopMotor();
        } else {
            leftPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
            rightPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
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
        double angleToSpeaker = FiringSolutionsV2.getAngleToSpeaker(pose.getX(), pose.getY());
        double robotVelocityTowardsSpeaker = FiringSolutionsV2.getRobotVelocityTowardsSpeaker(
                chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
                angleToSpeaker,
                pose.getRotation().getRadians());
        double distanceToSpeaker = FiringSolutionsV2.getDistanceToSpeaker(pose.getX(), pose.getY());
        double Cdeeznuts = FiringSolutionsV2.C(distanceToSpeaker, robotVelocityTowardsSpeaker);

        FiringSolutionsV2.updateR(
                FiringSolutionsV2.quarticA(robotVelocityTowardsSpeaker),
                FiringSolutionsV2.quarticB(distanceToSpeaker,
                        robotVelocityTowardsSpeaker),
                FiringSolutionsV2.quarticC(distanceToSpeaker,
                        robotVelocityTowardsSpeaker,
                        Cdeeznuts),
                FiringSolutionsV2.quarticD(distanceToSpeaker,
                        robotVelocityTowardsSpeaker,
                        Cdeeznuts),
                FiringSolutionsV2.quarticE(distanceToSpeaker,
                        robotVelocityTowardsSpeaker,
                        Cdeeznuts));

        shooterAngle = FiringSolutionsV2.getShooterAngle();
        // Calculate angle
        /*shooterAngle = FiringSolutions.getShooterAngle(
                FiringSolutions.getShooterVelocityX(pose.getX(), pose.getY()),
                velocityZ,
                FiringSolutions.getRobotVelocityTowardsSpeaker(
                        //                        chassisSpeeds.vxMetersPerSecond,
                        //                        chassisSpeeds.vyMetersPerSecond,
                        0, 0,
                        angleToSpeaker,
                        pose.getRotation().getRadians()));*/

        // Calculate velocity
        /*shooterVelocity = FiringSolutions.getShooterVelocity(
                FiringSolutions.getShooterVelocityX(pose.getX(), pose.getY()),
                velocityZ,
                FiringSolutions.getRobotVelocityTowardsSpeaker(
                        //                        chassisSpeeds.vxMetersPerSecond,
                        //                        chassisSpeeds.vyMetersPerSecond,
                        0, 0,
                        angleToSpeaker,
                        pose.getRotation().getRadians()),
                FiringSolutions.getRobotVelocityPerpendicularToSpeaker(
                        //                    chassisSpeeds.vxMetersPerSecond,
                        //                    chassisSpeeds.vyMetersPerSecond,
                        0, 0,
                        angleToSpeaker,
                        pose.getRotation().getRadians()));*/

        SmartDashboard.putNumber("Calculated Angle Set", shooterAngle);
        SmartDashboard.putNumber("distance", FiringSolutionsV2.getDistanceToSpeaker(pose.getX(), pose.getY()));
        //SmartDashboard.putNumber("Vx", FiringSolutions.getShooterVelocityX(pose.getX(), pose.getY()));
        //SmartDashboard.putNumber("Vz", velocityZ);
    }
}