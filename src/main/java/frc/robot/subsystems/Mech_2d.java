// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.*;

public class Mech_2d extends SubsystemBase {
  private ElevatorSubsystem m_elevatorSubsystem;
  private ShooterSubsystem m_shooter;
  /** Creates a new Mech_2d. */
    MechanismLigament2d  m_elevator;
    MechanismLigament2d m_wrist;
    Color8Bit m_wristColor;
      Mechanism2d mech = new Mechanism2d(3, 3);

  public Mech_2d(ShooterSubsystem shooter, ElevatorSubsystem elevator) {
    this.m_elevatorSubsystem = elevator;
    this.m_shooter = shooter;
      SmartDashboard.putData("Mech2d", mech);
      SmartDashboard.putData(this);
    // the main mechanism object

  
    // the mechanism root node

    MechanismRoot2d root = mech.getRoot("elevator", 2, 0);
    m_wristColor = new Color8Bit(Color.kPurple);

    // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based

    // off the root node or another ligament object

     m_elevator = root.append(new MechanismLigament2d("elevator", Constants.Elevator.maxHeightMeters, 90));

    m_wrist =

        m_elevator.append(

            new MechanismLigament2d("wrist", 0.5, 90,5,m_wristColor));
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_elevator.setLength(m_elevatorSubsystem.getHeight());

    m_wrist.setAngle(m_shooter.getCurrentShooterAngle());
         
  }


  }

