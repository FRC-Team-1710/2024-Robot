// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.FiringSolutions;

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
    public CANSparkBase m_Wrist = new CANSparkMax(3, MotorType.kBrushless);
    public CANSparkBase m_ShootaL = new CANSparkMax(7, MotorType.kBrushless); // leader
    public CANSparkBase m_ShootaR = new CANSparkMax(6, MotorType.kBrushless);
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
    private double setpointv = 0;
    private double setpointp = 0;
    private Boolean ENCFAIL;
    private double shooterVelocity;
    private double shooterAngle;
    public boolean isZeroed = false;

    private SwerveSubsystem swerveSubsystem;

    public ShooterSubsystem(SwerveSubsystem swerve) {
        swerveSubsystem = swerve;

        // Encoders
        m_VelocityEncoder = m_ShootaL.getEncoder();
        m_PositionEncoder = m_Wrist.getEncoder();
        m_WristEncoder = new DutyCycleEncoder(9);

        // Spark Max Setup
        m_ShootaL.restoreFactoryDefaults();
        m_ShootaR.restoreFactoryDefaults();
        m_Wrist.restoreFactoryDefaults();
        m_ShootaR.setInverted(true);

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

    public double getVelocity() {  
        return m_VelocityEncoder.getVelocity();
    }

    public double getAngle() {
        if (!ENCFAIL){
            return (m_WristEncoder.get() * 2 * Math.PI) / 4;
        } else {
            return (m_PositionEncoder.getPosition() * 2 * Math.PI) / 100;
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
        leftPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
        rightPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    public void PointShoot(double PointAngle, double launchVelocity) {
        m_Wrist.set(m_pidWrist.calculate(getAngle(), PointAngle));
        leftPID.setReference(launchVelocity, CANSparkMax.ControlType.kVelocity);
        rightPID.setReference(launchVelocity, CANSparkMax.ControlType.kVelocity);
    }

    public void manualWristSpeed(double speed){
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
        double velocityZ = FiringSolutions.getShooterVelocityZ();
        double angleToSpeaker = FiringSolutions.getAngleToSpeaker(pose.getX(), pose.getY());
        
        // Calculate angle
        shooterAngle = FiringSolutions.getShooterAngle(
                FiringSolutions.getShooterVelocityX(pose.getX(), pose.getY()),
                velocityZ,
                FiringSolutions.getRobotVelocityTowardsSpeaker(
//                        chassisSpeeds.vxMetersPerSecond,
//                        chassisSpeeds.vyMetersPerSecond,
0,0,
                        angleToSpeaker,
                        pose.getRotation().getRadians()));

        // Calculate velocity
        shooterVelocity = FiringSolutions.getShooterVelocity(
                FiringSolutions.getShooterVelocityX(pose.getX(), pose.getY()),
                velocityZ,
                FiringSolutions.getRobotVelocityTowardsSpeaker(
//                        chassisSpeeds.vxMetersPerSecond,
//                        chassisSpeeds.vyMetersPerSecond,
0,0,
                        angleToSpeaker,
                        pose.getRotation().getRadians()),
                FiringSolutions.getRobotVelocityPerpendicularToSpeaker(
//                    chassisSpeeds.vxMetersPerSecond,
//                    chassisSpeeds.vyMetersPerSecond,
0,0,
                    angleToSpeaker,
                    pose.getRotation().getRadians()));

        SmartDashboard.putNumber("Calculated Angle Set", shooterAngle);
        SmartDashboard.putNumber("distance", FiringSolutions.getDistanceToSpeaker(pose.getX(), pose.getY()));
        SmartDashboard.putNumber("Vx", FiringSolutions.getShooterVelocityX(pose.getX(), pose.getY()));
        SmartDashboard.putNumber("Vz", velocityZ);
        SmartDashboard.putNumber("Calculated Velocity Set", shooterVelocity);
        SmartDashboard.putNumber("Converted Velocity Set", FiringSolutions.convertToRPM(shooterVelocity));
    }
}