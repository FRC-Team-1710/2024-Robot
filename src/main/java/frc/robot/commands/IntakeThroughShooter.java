// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.IntexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeThroughShooter extends Command {
    private ShooterSubsystem shooter;
    private IntexerSubsystem intexer;
    private LEDSubsystem leds;

    Joystick controller;

    /** Creates a new IntakeFromShooter. */
    public IntakeThroughShooter(
            ShooterSubsystem shooterSub,
            IntexerSubsystem intex,
            LEDSubsystem LEDs,
            Joystick controller) {
        shooter = shooterSub;
        intexer = intex;
        leds = LEDs;
        this.controller = controller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSub, intex);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.setWristByAngle(.66);
        leds.sourceTele(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setShooterVelocity(-500);
        intexer.setALL(Constants.Intake.outakeSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelocity(Constants.Shooter.idleSpeedRPM);
        shooter.setWristByAngle(Constants.Shooter.intakeAngleRadians);
        intexer.setALL(0);
        leds.sourceTele(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (intexer.intakeBreak()) {
            controller.setRumble(RumbleType.kBothRumble, 0.75);
            return true;
        } else {
            return false;
        }
    }
}
