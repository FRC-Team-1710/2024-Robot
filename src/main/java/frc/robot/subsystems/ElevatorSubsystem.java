// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;

public class ElevatorSubsystem extends SubsystemBase {

    // Devices
    public TalonFX m_elevatorLeft; // left leader
    public TalonFX m_elevatorRight;
    public LaserCan lasercan;

    // Falcon stuff
    private final PositionDutyCycle lockPosition = new PositionDutyCycle(0);
    private final PIDController elevatorPID = new PIDController(3, 1, 0);

    // Constants IN METERS
    private final double spoolCircumference = 0.0508;
    private final double gearRatio = 17.33;

    // Vars
    private double revolutionCount;
    private double currentHeight;
    private double setHeight;
    private LaserCan.Measurement measurement;

    public boolean manualOverride = false;
    public boolean locked = false;
    public Timer timer = new Timer();

    public Orchestra m_orchestra = new Orchestra();

    public ElevatorSubsystem() {
        m_elevatorLeft = new TalonFX(21); // left leader
        m_elevatorRight = new TalonFX(20);
        lasercan = new LaserCan(22);

        // Falcon setup
        TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration();
        elevatorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorConfigs.Slot0.kP = 0.09;
        elevatorConfigs.Slot1.kP = 1;
        elevatorConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorConfigs.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.25;
        elevatorConfigs.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.25;
        elevatorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

        elevatorConfigs.Audio.AllowMusicDurDisable = true;

        m_elevatorLeft.getConfigurator().apply(elevatorConfigs);
        m_elevatorRight.getConfigurator().apply(elevatorConfigs);
        m_elevatorRight.setControl(new Follower(m_elevatorLeft.getDeviceID(), true));

        elevatorPID.setTolerance(0.02);

        try {
            lasercan.setRangingMode(RangingMode.SHORT);
        } catch (ConfigurationFailedException e) {
            DataLogManager.log(e.getMessage());
        }

        SmartDashboard.putData(this);
        SmartDashboard.putData("Elevator PID", elevatorPID);

        m_elevatorLeft.setPosition(0);

        // Add a single device to the orchestra
        m_orchestra.addInstrument(m_elevatorLeft, 1);
        m_orchestra.addInstrument(m_elevatorRight, 2);

        // Attempt to load the chrp
        var status = m_orchestra.loadMusic(Filesystem.getDeployDirectory()
                .toPath()
                .resolve("orchestra" + File.separator + "dangerzone.chrp")
                .toString());

        if (!status.isOK()) {
            // log error
        }

        // m_orchestra.play();
        m_orchestra.close();
        timer.reset();
        timer.start();
    }

    @Override
    public void periodic() {
        /*if (timer.get() > 5){
            if (m_orchestra.isPlaying()){
                m_orchestra.stop();
            }
            m_orchestra.close();
            timer.stop();
            timer.reset();
        }*/
        // This method will be called once per scheduler run
        updateHeightLaserCan();
        SmartDashboard.putNumber("Encoder Height", getHeightEncoder());
        SmartDashboard.putNumber("Encoder Raw", revolutionCount);
        SmartDashboard.putNumber("Set Height", setHeight);
        SmartDashboard.putNumber("LaserCan Meters", getHeightLaserCan());
        SmartDashboard.putBoolean("LaserCan failure", lasercanFailureCheck());
        SmartDashboard.putNumber(
                "Elevator Left Supply Current",
                m_elevatorLeft.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(
                "Elevator Right Supply Current",
                m_elevatorRight.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("LaserCan Ambient", measurement != null ? measurement.ambient : 0);
        revolutionCount = m_elevatorLeft.getPosition().getValueAsDouble();

        // FiringSolutionsV3.updateHeight(getHeight());
    }

    public void setElevatorSpeedManual(double value) {
        m_elevatorLeft.set(value);
    }

    public void setManualOverride(boolean value) {
        manualOverride = value;
    }

    public double getEncoderValue() {
        return revolutionCount;
    }

    public void setPositionWithEncoder(double value) {
        locked = true;
        m_elevatorLeft.setControl(lockPosition.withPosition(value));
    }

    public void stopHere() {
        locked = true;
        m_elevatorLeft.setControl(lockPosition.withPosition(revolutionCount).withSlot(0));
    }

    /** Get height from motor encoder */
    public double getHeightEncoder() {
        return (revolutionCount / gearRatio) * (spoolCircumference * Math.PI);
    }

    /**
     * Set height IN METERS. Will run off LaserCan but will switch to encoder if it
     * fails
     */
    public void setHeight(double height) {
        locked = true;
        setHeight = height;
        m_elevatorLeft.set(elevatorPID.calculate(getHeight(), height));
    }

    /**
     * Get height IN METERS. Will run off LaserCan but will switch to encoder if it
     * fails
     */
    public double getHeight() {
        if (!lasercanFailureCheck()) { // Run off LaserCan
            return getHeightLaserCan();
        } else { // Run off encoder
            return getHeightEncoder();
        }
    }

    public boolean atHeight() {
        return elevatorPID.atSetpoint();
    }

    public void ManSpin(double percent) {
        locked = false;
        m_elevatorLeft.set(percent);
    }

    public void resetElevatorEncoder() {
        m_elevatorLeft.setPosition(0);
    }

    /** Get height from LaserCan IN METERS */
    public double getHeightLaserCan() {
        return currentHeight;
    }

    public void updateHeightLaserCan() {
        measurement = lasercan.getMeasurement();
        if (measurement != null) {
            currentHeight = Double.valueOf(measurement.distance_mm) / 1000;
        }
    }

    /** Check if LaserCan has a result */
    public boolean lasercanFailureCheck() {
        if (measurement == null) {
            return true;
        } else {
            return false;
        }
    }

    public double getSetpoint() {
        return m_elevatorLeft.getClosedLoopReference().getValue();
    }

    public double getPosition() {
        return m_elevatorLeft.getPosition().getValueAsDouble();
    }
}
