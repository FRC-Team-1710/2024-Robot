// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    // Devices
    public TalonFX m_elevatorLeft = new TalonFX(20); // left leader
    public TalonFX m_elevatorRight = new TalonFX(21);
    public LaserCan lasercan = new LaserCan(22);
    
    // Falcon stuff
    private final Follower m_requestFollower = new Follower(0, true);
    private final PositionDutyCycle m_requestPosition = new PositionDutyCycle(0);
    private final PhoenixPIDController phoenixPID = new PhoenixPIDController(0, 0, 0);
    
    // Constants IN METERS
    private final double spoolCircumference = 0.0508;
    private final double gearRatio = 17.33;
    private final double maxHeight = 0;
    
    // Vars
    private double revolutionCount;
    private double setHeight;
    private boolean laser;
    LaserCan.Measurement measurement;
    
    public ElevatorSubsystem() {
        m_elevatorRight.setControl(m_requestFollower);

        Slot0Configs encoderConfig = new Slot0Configs();
        //ClosedLoopRampsConfigs closedloop = new ClosedLoopRampsConfigs();

        //closedloop.withDutyCycleClosedLoopRampPeriod(.2);
        encoderConfig.kP = 0.015;
        encoderConfig.kI = .0;
        encoderConfig.kD = 0.;
        encoderConfig.kV = .01;

        m_elevatorLeft.getConfigurator().apply(encoderConfig, 0.050);
        //m_elevatorLeft.getConfigurator().apply(closedloop);

        // laser can pid shenanigans
        phoenixPID.setP(0);
        phoenixPID.setI(0);
        phoenixPID.setD(0);

        laser = false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("height", getHeightEncoder());
        SmartDashboard.putNumber("Set Height", setHeight);
        SmartDashboard.putNumber("laserCan Raw", getHeightLaserCan());
        SmartDashboard.putBoolean("laserCan failure", lasercanFailureCheck());
        revolutionCount = m_elevatorLeft.getPosition().getValueAsDouble();
    }

    public double getHeightEncoder() {
        return (revolutionCount / gearRatio) * (spoolCircumference * Math.PI);
    }

    public void setHeight(double height) {
        setHeight = height;
        if (!lasercanFailureCheck()) { // Run off LaserCan
            m_elevatorLeft.set(phoenixPID.calculate(getHeightLaserCan(), height, .1));
            m_elevatorRight.set(phoenixPID.calculate(getHeightLaserCan(), height, .1));
        } else { // Run off encoder
            double rot = (height / (spoolCircumference * Math.PI)) * gearRatio;
            if (getHeightEncoder() < maxHeight) {
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

    public int getHeightLaserCan() {
        if (measurement != null) {
            return measurement.distance_mm / 1000; //UNITS MATTER!!!! METERS ONLY!!!!
        }
        return 0;
    }

    public boolean lasercanFailureCheck() {
        if (measurement == null) {
            return true;
        } else {
            return false;
        }
    }

}
