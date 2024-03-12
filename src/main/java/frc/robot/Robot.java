// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Optional;
import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.math.FiringSolutionsV3;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {
    public static final CTREConfigs ctreConfigs = new CTREConfigs();

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private static boolean redAlliance;

    //PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        // Disable LiveWindow since we don't use it
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

        DriverStation.silenceJoystickConnectionWarning(true);

        // Starts recording to data log
        DataLogManager.start("/media/sda1/logs/", DateTimeFormatter.ofPattern("yyyy-MM-dd__HH-mm-ss").format(LocalDateTime.now())+".wpilog");

        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.log(
                "\nF  I  R  S  T    R  O  B  O  T  I  C  S    T  E  A  M\n______________   _  _____   _  _____   ______________\n\\_____________| / ||___  | / ||  _  | |_____________/\n \\_ _ _ _ _ _ | | |   / /  | || | | | | _ _ _ _ _ _/\n  \\ _ _ _ _ _ | | |  / /   | || |_| | | _ _ _ _ _ /\n   \\__________|_|_|_/_/___ |_||_____|_|__________/\n    \\____________________/ \\____________________/\n");

        // Log data from all REV devices
        URCL.start();

        // Log data from all CTRE devices
        SignalLogger.setPath("/media/sda1/logs/");
        SignalLogger.start();

        // Output command scheduler to dashboard
        SmartDashboard.putData(CommandScheduler.getInstance());

        // Access PhotonVision dashboard when connected via usb TODO make work
       // PortForwarder.add(5800, "10.17.10.11", 5800);

       //SmartDashboard.putData(PDH);

       // idk if this is useful
        System.gc();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** Gets the current alliance, true is red */
    public static boolean getAlliance() {
        return redAlliance;
    }

    public static boolean checkRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        m_robotContainer.stopAll();

        if (m_autonomousCommand != null){
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        redAlliance = checkRedAlliance();
    
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        FiringSolutionsV3.setAlliance(redAlliance);

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        FiringSolutionsV3.resetAllR();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        redAlliance = checkRedAlliance();

        FiringSolutionsV3.setAlliance(redAlliance);
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        FiringSolutionsV3.resetAllR();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
