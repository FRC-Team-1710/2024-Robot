package frc.robot;

import java.lang.annotation.Target;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.math.FiringSolutionsV3;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick mech = new Joystick(1);

    /* Analog */
    private final int leftVerticalAxis = XboxController.Axis.kLeftY.value;
    private final int leftHorizontalAxis = XboxController.Axis.kLeftX.value;
    private final int rightHorizontalAxis = XboxController.Axis.kRightX.value;
    private final int rightVerticalAxis = XboxController.Axis.kRightY.value;
    private final int leftTrigger = XboxController.Axis.kLeftTrigger.value;
    private final int rightTrigger = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton resetOdom = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton Shoot = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton intex = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton outex = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton xButtonDriver = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton forceShoot = new JoystickButton(driver, XboxController.Button.kY.value);
    private final Trigger targetAmp = new Trigger(() -> driver.getRawAxis(leftTrigger) > .5);
    private final Trigger targetSpeaker = new Trigger(() -> driver.getRawAxis(rightTrigger) > .5);

    /* Mech Buttons */
    private final JoystickButton zeroShooter = new JoystickButton(mech, XboxController.Button.kBack.value);
    private final JoystickButton primeShooterSpeedSpeaker = new JoystickButton(mech, XboxController.Button.kRightBumper.value);
    private final JoystickButton primeShooterSpeedAmp = new JoystickButton(mech, XboxController.Button.kLeftBumper.value);
    private final JoystickButton shooterTo45 = new JoystickButton(mech, XboxController.Button.kB.value);
    private final JoystickButton shooterToIntake = new JoystickButton(mech, XboxController.Button.kA.value);
    private final Trigger resetR = new Trigger(() -> mech.getPOV() == 90);
    private final Trigger elevatorDown = new Trigger(() -> mech.getPOV() == 180);
    private final Trigger elevatorUp = new Trigger(() -> mech.getPOV() == 0);
    private final Trigger intakeThroughShooter = new Trigger(() -> mech.getPOV() == 270);
    private final JoystickButton autoZeroShooter = new JoystickButton(mech, XboxController.Button.kStart.value);
    private final JoystickButton shooterToAmp = new JoystickButton(mech, XboxController.Button.kY.value);
    private final JoystickButton xButtonMech = new JoystickButton(mech, XboxController.Button.kX.value);
    private final JoystickButton rightStick = new JoystickButton(mech, XboxController.Button.kRightStick.value);

    /* 
    private final Trigger dynamicForward = new Trigger(() -> mech.getPOV() == 90);
    private final Trigger dynamicBackward = new Trigger(() -> mech.getPOV() == 270);
    private final Trigger quasistaticForward = new Trigger(() -> mech.getPOV() == 0);
    private final Trigger quasistaticBackwards = new Trigger(() -> mech.getPOV() == 180);
    */

    /* Subsystems */
    private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
    private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(m_VisionSubsystem);
    public final ShooterSubsystem m_Shoota = new ShooterSubsystem(m_SwerveSubsystem);
    private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem(m_VisionSubsystem, m_Shoota);
    //private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem(m_VisionSubsystem);
    private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    private final IntexerSubsystem m_IntexerSubsystem = new IntexerSubsystem();

    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_SwerveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        m_SwerveSubsystem, m_VisionSubsystem, m_Shoota,
                        () -> -driver.getRawAxis(leftVerticalAxis),
                        () -> -driver.getRawAxis(leftHorizontalAxis),
                        () -> -driver.getRawAxis(rightHorizontalAxis),
                        () -> robotCentric.getAsBoolean(),
                        () -> targetAmp.getAsBoolean(),
                        () -> targetSpeaker.getAsBoolean(),
                        () -> intex.getAsBoolean(), driver));

        m_ElevatorSubsystem.setDefaultCommand(
                new ElevationManual(
                        m_ElevatorSubsystem,
                        () -> -mech.getRawAxis(leftVerticalAxis)));

        m_Shoota.setDefaultCommand(new ManRizzt(m_Shoota, m_LEDSubsystem, () -> -mech.getRawAxis(rightVerticalAxis), () -> shooterTo45.getAsBoolean()));

        // m_LEDSubsystem.setAllianceColor();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */ 
    private void configureButtonBindings() {
        /* DRIVER BUTTONS */

        // Lock on to speaker
        targetSpeaker.whileTrue(new MissileLock(m_Shoota, "speaker"));
        targetAmp.whileTrue(new MissileLock(m_Shoota, "amp"));
        
        // Shooter
        targetSpeaker.or(targetAmp).and(Shoot).whileTrue(new FIREEE(m_Shoota, m_IntexerSubsystem, m_LEDSubsystem)); // Main fire

        // Reset Odometry
        resetOdom.onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroHeading()).alongWith(
                new InstantCommand(() -> m_SwerveSubsystem.setPose(new Pose2d(1.35, 5.55, new Rotation2d(0))))));

        // Intexer
        intex.whileTrue(new IntexBestHex(m_IntexerSubsystem, true, driver));
        outex.whileTrue(new IntexBestHex(m_IntexerSubsystem, false, driver));

        // Shooter intake
        forceShoot.whileTrue(new InstantCommand(() -> m_IntexerSubsystem.setShooterIntake(.9)))
        .onFalse(new InstantCommand(() -> m_IntexerSubsystem.setShooterIntake(0)));
        
        /* MECH BUTTONS */
        
        // Prime for Speaker
        primeShooterSpeedSpeaker
        .whileTrue(new InstantCommand(() -> m_Shoota.setShooterVelocity(FiringSolutionsV3.convertToRPM(m_Shoota.getCalculatedVelocity()))))
        .onFalse(new InstantCommand(() -> m_Shoota.setShooterVelocity(Constants.Shooter.idleSpeedRPM)));
        
        // Prime for Amp
        primeShooterSpeedAmp.whileTrue(new InstantCommand(() -> m_Shoota.setShooterVelocity(3417.8)))
        .onFalse(new InstantCommand(() -> m_Shoota.setShooterVelocity(Constants.Shooter.idleSpeedRPM)));
        
        // Elevator
        elevatorDown.onTrue(new ElevatorSet(m_ElevatorSubsystem, Constants.Elevator.minHeightMeters));
        elevatorUp.onTrue(new ElevatorSet(m_ElevatorSubsystem, Constants.Elevator.maxHeightMeters));
        
        // Zero wrist
        zeroShooter.onTrue(new InstantCommand(() -> m_Shoota.resetWristEncoders(Constants.Shooter.angleOffsetManual))); // Set encoder to zero
        autoZeroShooter.onTrue(new ZeroWrist(m_Shoota).andThen(new RizzLevel(m_Shoota, Constants.Shooter.intakeAngleRadians)));
        
        // Wrist
        shooterToIntake.onTrue(new RizzLevel(m_Shoota, Constants.Shooter.intakeAngleRadians)); // Move wrist to intake position
        
        // Amp Preset
        shooterToAmp.onTrue(new RizzLevel(m_Shoota, -0.48)).onTrue(new ElevatorSet(m_ElevatorSubsystem, Constants.Elevator.maxHeightMeters));
        
        //xButton.whileTrue(new InstantCommand(() -> m_Shoota.SetOffsetVelocity(2000)))
        //.onFalse(new InstantCommand(() -> m_Shoota.SetShooterVelocity(Constants.Shooter.idleSpeedRPM)));
        
        // Reset R
        resetR.onTrue(new InstantCommand(() -> FiringSolutionsV3.resetAllR())); // Reset the R calculation incase it gets off

        // Kill Shooter
        rightStick.onTrue(new InstantCommand(()-> m_Shoota.setShooterVelocity(0)));

        //Intake Through Shooter
        intakeThroughShooter.whileTrue(new IntakeThroughShooter(m_Shoota, m_IntexerSubsystem, mech))
        .onFalse(new IntakeThroughShooterPart2(m_Shoota, m_IntexerSubsystem, mech));

        // Characterization tests 
        /*dynamicForward.whileTrue(m_SwerveSubsystem.sysIdDynamic(Direction.kForward));
        dynamicBackward.whileTrue(m_SwerveSubsystem.sysIdDynamic(Direction.kReverse));
        quasistaticForward.whileTrue(m_SwerveSubsystem.sysIdQuasistatic(Direction.kForward));
        quasistaticBackwards.whileTrue(m_SwerveSubsystem.sysIdQuasistatic(Direction.kReverse));*/
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
