// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevatorSubsystem extends SubsystemBase {
    /** Creates a new elevatorSubsystem. */
    LaserCan lasercan = new LaserCan(39);
    LaserCan.Measurement measurement = lasercan.getMeasurement();
    public boolean laser;
    PhoenixPIDController pidSet = new PhoenixPIDController(0, 0, 0);
    // falcon

    public TalonFX m_elevatorLeft = new TalonFX(0); // left leader
    public TalonFX m_elevatorRight = new TalonFX(1);

    // requests
    final Follower m_requestFollower = new Follower(0, true);
    final VelocityDutyCycle m_requestVelocity = new VelocityDutyCycle(0);
    final PositionDutyCycle m_requestPosition = new PositionDutyCycle(0);
    // math
    double revolutionCount;
    double revolutioncircumphrence = 6.28;
    double setHeight;
    double maxHeightInCm;
    double offset;

    public elevatorSubsystem() {
      
        // magnetSensorConfigs.withMagnetOffset(offset);
        m_elevatorRight.setControl(m_requestFollower);

        var slot0Configs = new Slot0Configs();
        var closedloop = new ClosedLoopRampsConfigs();
        closedloop.withDutyCycleClosedLoopRampPeriod(.2);
        slot0Configs.kP = 0.015;
        slot0Configs.kI = .0;
        slot0Configs.kD = 0.;
        slot0Configs.kV = .01;

        // m_elevatorLeft.getConfigurator().apply(magnetSensorConfigs);
        m_elevatorLeft.getConfigurator().apply(slot0Configs, 0.050);

        // laser can pid shenanigans
        pidSet.setP(0);
        pidSet.setI(0);
        pidSet.setD(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("height", getHeightFromEncoder());
        SmartDashboard.putNumber("setPoint", setHeight);
        SmartDashboard.putNumber("laser Can Raw", getHeight());
        SmartDashboard.putBoolean("laserCan failure", lasercanFailureCheck());
        revolutionCount = m_elevatorLeft.getPosition().getValueAsDouble();
    }

    // encoder getters
    public double getHeightFromEncoder() {
        return (revolutionCount * revolutioncircumphrence)/17.04;
    }

    public void setHeight(double height) {
        setHeight = height;
        if (laser) {
            m_elevatorLeft.set(pidSet.calculate(getHeight(), height, .1));
            m_elevatorRight.set(pidSet.calculate(getHeight(), height, .1));
        } else {
            double rot = height / revolutioncircumphrence;
            if (getHeightFromEncoder() < maxHeightInCm) {
                m_elevatorLeft.setControl(m_requestPosition.withPosition(rot));
            }
        }
    }

    public void ManSpin(double percent) {
        m_elevatorLeft.set(percent);
    }

    public void resetElevatorEncoder() {
        m_elevatorLeft.setPosition(0);
    }

    //lasercan methods
    public void useLaserCan(boolean laserCanOn) {
        laser = laserCanOn;
    }

    public int getHeight() {
        if (measurement != null) {
            return measurement.distance_mm;
        }
        return 0;
    }

    public boolean lasercanFailureCheck() {
        if (measurement == null) {
            return true;
        } else {
            laser = false;
            return false;
        }
    }

}
