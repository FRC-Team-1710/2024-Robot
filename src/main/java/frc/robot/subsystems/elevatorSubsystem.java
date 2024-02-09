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
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevatorSubsystem extends SubsystemBase {
    /** Creates a new elevatorSubsystem. */
    LaserCan lasercan = new LaserCan(0);
    LaserCan.Measurement measurement = lasercan.getMeasurement();
    boolean laser;
    PhoenixPIDController pidSet = new PhoenixPIDController(0, 0, 0);
    // falcon
    public CoreTalonFX m_elevatorLeft = new CoreTalonFX(0); // left leader
    public TalonFX vaderLeft = new TalonFX(0);
    public CoreTalonFX m_elevatorRight = new CoreTalonFX(1);
    public TalonFX vaderRight = new TalonFX(1);
    // requests
    final Follower m_requestFollower = new Follower(0, true);
    final VelocityDutyCycle m_requestVelocity = new VelocityDutyCycle(0);
    final PositionDutyCycle m_requestPosition = new PositionDutyCycle(0);
    // math
    double revolutionCount = m_elevatorLeft.getPosition().getValueAsDouble();
    double revolutioncircumphrence = 6.28;
    double setHeight;
    double maxHeightInCm;

    public elevatorSubsystem() {
        m_elevatorLeft = vaderLeft;
        m_elevatorRight = vaderRight;
        m_elevatorRight.setControl(m_requestFollower);
        vaderRight.setInverted(true);

        var slot0Configs = new Slot0Configs();
        var closedloop = new ClosedLoopRampsConfigs();
        closedloop.withDutyCycleClosedLoopRampPeriod(.2);
        slot0Configs.kP = 0.015;
        slot0Configs.kI = .0;
        slot0Configs.kD = 0.;
        slot0Configs.kV = .01;
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
    }

    // encoder getters

    public double getHeightFromEncoder() {
        return (revolutionCount * revolutioncircumphrence);
    }

    // encoder setters
    private void setElevatorVelocity(double velocity) {
        m_elevatorLeft.setControl(m_requestVelocity.withVelocity(velocity));
    }

    public void setHeightFromEncoder(double height) {
        setHeight = height;
        double rot = height / revolutioncircumphrence;
        if (getHeightFromEncoder() < height) {
            m_elevatorLeft.setControl(m_requestPosition.withPosition(rot));
        }
    }

    public void ManSpin(double percent) {
        vaderLeft.set(percent);
        vaderRight.set(percent);
    }

    public void resetElevatorEncoder() {
        // m_elevatorLeft
    }

    // laser can methods scrapped
    public void setHeightFromLaserCan(double height) {
        vaderLeft.set(pidSet.calculate(getHeight(), height, .1));
        vaderRight.set(pidSet.calculate(getHeight(), height, .1));
    }

    public void useLaserCan(boolean laserCanOn) {
        laser = laserCanOn;
    }

    public int getHeight() {
        if (measurement != null) {
            return measurement.distance_mm;
        }
        return 0;
    }

}
