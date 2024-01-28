// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.math.controller.PIDController;

public class ShootaTest extends SubsystemBase {

    public CANSparkBase m_Wrist = new CANSparkMax(3, MotorType.kBrushless);
    public CANSparkBase m_ShootaL = new CANSparkMax(7, MotorType.kBrushless); // leader
    public CANSparkBase m_ShootaR = new CANSparkMax(6, MotorType.kBrushless);
    private RelativeEncoder m_VelocityEncoder;
    public PIDController m_pidWrist; // create PIDController
    private SparkPIDController leftPID; // create PIDController
    private SparkPIDController rightPID;
    private DutyCycleEncoder m_WristEncoder; // create encoder
    private double setpointv = 0;
    private double setpointp = 0;
    private Boolean ENCFAIL;
    public double getA;

    private double pidSpdP = .0000002;
    private double pidSpdI = .0000002;
    private double pidSpdD = .006;

    private double pidPosP = .01;
    private double pidPosI = 0;
    private double pidPosD = 0;

    private double spin = 0;

    double getH;

    public ShootaTest() {
        m_VelocityEncoder = m_ShootaL.getEncoder();
        m_WristEncoder = new DutyCycleEncoder(9);
        m_ShootaL.restoreFactoryDefaults();
        m_ShootaR.restoreFactoryDefaults();
        m_Wrist.restoreFactoryDefaults();
//        m_ShootaR.follow(m_ShootaL, false);
        m_ShootaR.setInverted(true);
        leftPID = m_ShootaL.getPIDController();
        rightPID = m_ShootaR.getPIDController();

        // wrist
        m_pidWrist = new PIDController(pidPosP, pidPosI, pidPosD);

        SmartDashboard.putNumber("set velocity", 0);
        SmartDashboard.putNumber("set angle", 0);

        SmartDashboard.putNumber("Velo P", pidSpdP);
        SmartDashboard.putNumber("Velo I", pidSpdI);
        SmartDashboard.putNumber("Velo D", pidSpdD);
        SmartDashboard.putNumber("Spin", spin);

        resetEncoder();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        pidSpdP = SmartDashboard.getNumber("Velo P", pidSpdP);
        pidSpdI = SmartDashboard.getNumber("Velo I", pidSpdI);
        pidSpdD = SmartDashboard.getNumber("Velo D", pidSpdD);

        leftPID.setP(pidSpdP, 0);
        leftPID.setI(pidSpdI, 0);
        leftPID.setD(pidSpdD, 0);

        rightPID.setP(pidSpdP, 0);
        rightPID.setI(pidSpdI, 0);
        rightPID.setD(pidSpdD, 0);

        spin = SmartDashboard.getNumber("Spin", spin);

        setpointv = SmartDashboard.getNumber("set velocity", setpointv);
        setpointp = SmartDashboard.getNumber("set angle", setpointp);
        SmartDashboard.putNumber("current angle", getAngle());
        SmartDashboard.putNumber("current velocity", getVelocity());

        if (getAngle() == 0) {
            ENCFAIL = true;
        } else {
            ENCFAIL = false;
        }
        SmartDashboard.putBoolean("ODER FAILURE", ENCFAIL);

        //SetShooterVelocity(setpointv);
        //wristManualSet(setpointp);
    }

    public double getVelocity() {
        return m_VelocityEncoder.getVelocity();
    }

    public double getAngle() {
        //getA = ((m_WristEncoder.get() * 360));
        return m_WristEncoder.get() * 120;
    }

    public void resetEncoder() {
        m_WristEncoder.reset();
    }

    public void wristManualSet(double angle) {
        // double x = (42 / 360) * angle;
        m_Wrist.set(m_pidWrist.calculate(getAngle(), angle));
    }

    public void WristAngleSetFromSmartDashboard() {
        if (getAngle() > setpointp + 360) {
            m_pidWrist.setPID(0, 0, 0);
        } else {
            m_pidWrist.setPID(pidPosP, pidPosI, pidPosD);
        }
        m_Wrist.set(m_pidWrist.calculate(getAngle(), setpointp));
    }

    public void SetShooterVelocity(double velocity) {
        leftPID.setReference(velocity + spin, CANSparkMax.ControlType.kVelocity);
        rightPID.setReference(velocity - spin, CANSparkMax.ControlType.kVelocity);
    }


    public void PointShoot(double PointAngle, double launchVelocity) {
        if (getAngle() > setpointp + 360) {
            m_pidWrist.setPID(0, 0, 0);
        } else {
            m_pidWrist.setPID(pidPosP, pidPosI, pidPosD);
        }
        m_Wrist.set(m_pidWrist.calculate(getAngle(), PointAngle));
        leftPID.setReference(launchVelocity, CANSparkMax.ControlType.kVelocity);
        rightPID.setReference(launchVelocity, CANSparkMax.ControlType.kVelocity);
    }

    public void sillyString(double speed) {
        m_ShootaL.set(speed);
    }

    public void StartShoota() {
        resetEncoder();
        wristManualSet(0);
    }

    public void manualWristSpeed(double speed){
        m_Wrist.set(speed);
    }

}