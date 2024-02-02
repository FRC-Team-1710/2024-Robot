package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
    //  private final Joystick mech = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton Shoot = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton wristUp = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton wristDown = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton zeroShooter = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Subsystems */
    private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
    private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(m_VisionSubsystem);
    public final ShooterSubsystem m_Shoota = new ShooterSubsystem(m_SwerveSubsystem);
    private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem(m_VisionSubsystem);

    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_SwerveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        m_SwerveSubsystem,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        m_LEDSubsystem.setAllianceColor();

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
        /* Driver Buttons */
        Shoot.whileTrue(new FIREEE(m_Shoota));
        zeroGyro.onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroHeading()));
        wristUp.whileTrue(new ManRizzt(m_Shoota, .05));
        wristDown.whileTrue(new ManRizzt(m_Shoota, -.05));
        zeroShooter.onTrue(new InstantCommand(() -> m_Shoota.resetWristEncoder()));

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
