// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntexerSubsystem extends SubsystemBase {

    // Devices
    private CANSparkBase left = new CANSparkMax(30, MotorType.kBrushless);
    private CANSparkBase right = new CANSparkMax(31, MotorType.kBrushless);
    private CANSparkBase shooterIntake = new CANSparkMax(10, MotorType.kBrushless);

    private DigitalInput breakingBeam = new DigitalInput(2);
    private DigitalInput beamKamen = new DigitalInput(1);

    public IntexerSubsystem() {

    }

    public void setALL(double speed) {
        if (speed == 0) {
            left.stopMotor();
            right.stopMotor();
            shooterIntake.stopMotor();
        } else {
            left.set(speed);
            right.set(speed);
            shooterIntake.set(-speed);
        }
    }

    public void setFrontIntake(double speed) {
        if (speed == 0) {
            left.stopMotor();
            right.stopMotor();
        } else {
            left.set(speed);
            right.set(speed);
        }
    }

    public void setShooterIntake(double speed) {
        if (speed == 0){
            shooterIntake.stopMotor();
        } else {
            shooterIntake.set(-speed);
        }
    }

    public boolean intakeBreak () {
        return breakingBeam.get();
    }

    public boolean shooterBreak() {
        return beamKamen.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Beam Break", breakingBeam.get());
        SmartDashboard.putBoolean("Shooter Beam Break", beamKamen.get());
    }
}
