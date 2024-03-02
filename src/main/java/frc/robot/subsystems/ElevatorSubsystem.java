// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    // Devices
    public TalonFX m_elevatorLeft = new TalonFX(20); // left leader
    public TalonFX m_elevatorRight = new TalonFX(21);
    public LaserCan lasercan = new LaserCan(22);

    // Falcon stuff
    private final PositionDutyCycle m_requestPosition = new PositionDutyCycle(0);
    private final PositionDutyCycle lockPosition = new PositionDutyCycle(0);
    private final PIDController elevatorPID = new PIDController(0, 0, 0);
    private final Slot1Configs encoderConfigSlot1 = new Slot1Configs();

    // Constants IN METERS
    private final double spoolCircumference = 0.0508;
    private final double gearRatio = 17.33;
    private final double maxHeight = .8;

    // Vars
    private double revolutionCount;
    private double currentHeight;
    private double setHeight;
    private boolean laser;
    LaserCan.Measurement measurement;

    public boolean manualOverride = false;
    public boolean locked = false;

    public ElevatorSubsystem() {
        // Falcon setup
        m_elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
        m_elevatorRight.setControl(new Follower(m_elevatorLeft.getDeviceID(), true));

        // PID
        Slot0Configs encoderConfigSlot0 = new Slot0Configs();
        ClosedLoopRampsConfigs closedloop = new ClosedLoopRampsConfigs();

        closedloop.withDutyCycleClosedLoopRampPeriod(.5).withTorqueClosedLoopRampPeriod(0.5);
        encoderConfigSlot0.kP = .09;
        encoderConfigSlot0.kI = .0;
        encoderConfigSlot0.kD = .0;
        encoderConfigSlot0.kV = .0;
        encoderConfigSlot0.kG = .0;
        encoderConfigSlot0.GravityType = GravityTypeValue.Elevator_Static;

        encoderConfigSlot1.kP = 1;
        encoderConfigSlot1.kI = .0;
        encoderConfigSlot1.kD = .0;
        encoderConfigSlot1.kV = .01;


        m_elevatorLeft.getConfigurator().apply(encoderConfigSlot0, 0.050);
        m_elevatorLeft.getConfigurator().apply(closedloop);

        // laser can pid shenanigans
        elevatorPID.setP(2.5);
        elevatorPID.setI(0);
        elevatorPID.setD(0);
        elevatorPID.setTolerance(0.02);

        try {
            lasercan.setRangingMode(RangingMode.SHORT);
        } catch (ConfigurationFailedException e) {
            DataLogManager.log(e.getMessage());
        }

        SmartDashboard.putData(this);
        SmartDashboard.putData("Elevator PID", elevatorPID);

        m_elevatorLeft.setPosition(0);

        laser = false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateHeightLaserCan();
        SmartDashboard.putNumber("Encoder Height", getHeightEncoder());
        SmartDashboard.putNumber("Encoder Raw", revolutionCount);
        SmartDashboard.putNumber("Set Height", setHeight);
        SmartDashboard.putNumber("LaserCan Meters", getHeightLaserCan());
        SmartDashboard.putBoolean("LaserCan failure", lasercanFailureCheck());
        SmartDashboard.putNumber("Elevator Left Supply Current", m_elevatorLeft.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Right Supply Current",
        m_elevatorRight.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("LaserCan Ambient", measurement.ambient);
        revolutionCount = m_elevatorLeft.getPosition().getValueAsDouble();

        if (!manualOverride){ // Flagitious logic
            if (!locked){
                stopHere();
                locked = true;
            }
        } else {
            locked = false;
        }
        
        //FiringSolutionsV3.updateHeight(getHeight()); //TODO: test this
    }

    public void setManualOverride(boolean value){
        manualOverride = value;
    }

    public double getEncoderValue(){
        return revolutionCount;
    }

    public void setToEncoder(double value){
        m_elevatorLeft.setControl(lockPosition.withPosition(value));
    }

    public void stopHere(){
        m_elevatorLeft.setControl(lockPosition.withPosition(revolutionCount).withSlot(0));
    }

    /** Get height from motor encoder */
    public double getHeightEncoder() {
        return (revolutionCount / gearRatio) * (spoolCircumference * Math.PI) + 0.2;
    }

    /** Set height IN METERS. Will run off LaserCan but will switch to encoder if it fails */
    public void setHeight(double height) {
        setHeight = height;
        if (!lasercanFailureCheck()) { // Run off LaserCan
            m_elevatorLeft.set(elevatorPID.calculate(getHeightLaserCan(), height));
        } else { // Run off encoder
            double rot = (height / (spoolCircumference * Math.PI)) * gearRatio;
            if (getHeightEncoder() < maxHeight) {
                m_elevatorLeft.setControl(m_requestPosition.withPosition(rot).withSlot(1));
            }
        }
    }

    /** Get height IN METERS. Will run off LaserCan but will switch to encoder if it fails */
    public double getHeight(){
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
        m_elevatorLeft.set(percent);
    }

    public void resetElevatorEncoder() {
        m_elevatorLeft.setPosition(0);
    }

    //lasercan methods
    public void useLaserCan(boolean laserCanOn) {
        laser = laserCanOn;
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

}
