// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.*;
import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;

public class ShootaTest extends SubsystemBase {

  public CANSparkBase m_Wrist = new CANSparkMax(3, MotorType.kBrushless);
  public CANSparkBase m_ShootaL = new CANSparkMax(7, MotorType.kBrushless); // leader
  public CANSparkBase m_ShootaR = new CANSparkMax(6, MotorType.kBrushless);
  public CANSparkBase m_Intake = new CANSparkMax(5, MotorType.kBrushless); // leader
  public static PIDController m_pidWrist; // create PIDController
  private static SparkPIDController m_pidShoota; // create PIDController
  private static SparkPIDController m_pidIntake; // create PIDController
  private static DutyCycleEncoder m_WristEncoder; // create encoder
  private double setpointv = 0;
  private double setpointp = 0;
  private Boolean ENCFAIL;
  public double getA;

  int pidPosP = 0;
  int pidPosI = 0;
  int pidPosD = 0;
  double getH;

  /** Creates a new IntakeNWrist. */

  public ShootaTest() {

    m_WristEncoder = new DutyCycleEncoder(1);
    m_Intake.restoreFactoryDefaults();
    m_ShootaL.restoreFactoryDefaults();
    m_ShootaR.restoreFactoryDefaults();
    m_Wrist.restoreFactoryDefaults();
    m_ShootaR.follow(m_ShootaL);
    m_ShootaR.setInverted(true);
    m_pidShoota = m_ShootaL.getPIDController();
    // m_pidIntake = m_IntakeL.getPIDController();
    int pidPosP = 0;
    int pidPosI = 0;
    int pidPosD = 0;

    double pidSpdP = .0000002;
    double pidSpdI = .0000004;
    double pidSpdD = .0016;
    // wrist
    m_pidWrist = new PIDController(pidPosP, pidPosI, pidPosD);
    // Shoota
    m_pidShoota.setP(pidSpdP, 0);
    m_pidShoota.setI(pidSpdI, 0);
    m_pidShoota.setD(pidSpdD, 0);
    // intake
    // m_pidIntake.setP(pidSpdP,0);
    // m_pidIntake.setI(pidSpdI,0);
    // m_pidIntake.setD(pidSpdD,0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getNumber("velocity", setpointv);
    SmartDashboard.getNumber("angle", setpointp);
    SmartDashboard.putNumber("encoder angle", getAngle());
    if (getAngle() == 0) {
      ENCFAIL = true;
    } else {
      ENCFAIL = false;
    }
    SmartDashboard.putBoolean("ENCODER FAILURE", ENCFAIL);
  }

  public double getAngle() {
    getA = ((m_WristEncoder.get() * 120));
    return getA;
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

  public void Intaking(double speed) {
    m_pidIntake.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public void ManualShootaSPD(double velocity) {
    m_pidShoota.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void SetVelocityFromDashboard() {
    m_pidShoota.setReference(setpointv, CANSparkMax.ControlType.kVelocity);
  }

  public void PointShoot(double PointAngle, double launchVelocity) {
  if (getAngle() > setpointp + 360) {
      m_pidWrist.setPID(0, 0, 0);
    } else {
      m_pidWrist.setPID(pidPosP, pidPosI, pidPosD);
    }
    m_Wrist.set(m_pidWrist.calculate(getAngle(), PointAngle));
      m_pidShoota.setReference(launchVelocity, CANSparkMax.ControlType.kVelocity);
  }

  public void sillyString(double speed) {
    m_ShootaL.set(speed);

  }

  public void StartShoota() {
    resetEncoder();
    wristManualSet(0);
  }

}