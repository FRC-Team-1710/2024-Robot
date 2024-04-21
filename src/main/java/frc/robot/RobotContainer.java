package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick mech = new Joystick(1);
    private final Joystick FF = new Joystick(2);

    /* Analog */
    private final int leftVerticalAxis = XboxController.Axis.kLeftY.value;
    private final int leftHorizontalAxis = XboxController.Axis.kLeftX.value;
    private final int rightHorizontalAxis = XboxController.Axis.kRightX.value;
    private final int rightVerticalAxis = XboxController.Axis.kRightY.value;
    private final int leftTrigger = XboxController.Axis.kLeftTrigger.value;
    private final int rightTrigger = XboxController.Axis.kRightTrigger.value;

    /* DRIVER BUTTONS spotless:off */
    /** Driver X */
    private final JoystickButton intakeNoMove = new JoystickButton(driver, XboxController.Button.kX.value);
    /** Driver A */
    private final JoystickButton Shoot = new JoystickButton(driver, XboxController.Button.kA.value);
    /** Driver B */
    private final JoystickButton forceShoot = new JoystickButton(driver, XboxController.Button.kB.value);
    /** Driver Y */
    private final JoystickButton intakeFromSource = new JoystickButton(driver, XboxController.Button.kY.value);
    /** Driver RB */
    private final JoystickButton intex = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    /** Driver LB */
    private final JoystickButton outex = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    /** Driver LT */
    private final Trigger targetAmp = new Trigger(() -> driver.getRawAxis(leftTrigger) > .5);
    /** Driver RT */
    private final Trigger targetSpeaker = new Trigger(() -> driver.getRawAxis(rightTrigger) > .5);
    /** Driver Up */
    private final Trigger driverUp = new Trigger(() -> driver.getPOV() == 0);
    /** Driver Down */
    private final Trigger driverDown = new Trigger(() -> driver.getPOV() == 180);
    /** Driver Right */
    private final Trigger driverLeft = new Trigger(() -> driver.getPOV() == 90);
    /** Driver Left */
    private final Trigger driverRight = new Trigger(() -> driver.getPOV() == 270);
    /** Driver Start */
    private final JoystickButton resetOdom = new JoystickButton(driver, XboxController.Button.kStart.value);

    /* MECH BUTTONS */
    /** Mech X */
    private final JoystickButton shooterToAntiDefense = new JoystickButton(mech, XboxController.Button.kX.value);
    /** Mech A */
    private final JoystickButton shooterToIntake = new JoystickButton(mech, XboxController.Button.kA.value);
    /** Mech Y */
    private final JoystickButton shooterToAmp = new JoystickButton(mech, XboxController.Button.kY.value);
    /** Mech B */
    private final JoystickButton shooterToSubwoofer = new JoystickButton(mech, XboxController.Button.kB.value);
    /** Mech RB */
    private final JoystickButton primeShooterSpeedSpeaker = new JoystickButton(mech, XboxController.Button.kRightBumper.value);
    /** Mech LB */
    private final JoystickButton resetNoteInShooter = new JoystickButton(mech, XboxController.Button.kLeftBumper.value);
    /** Mech RT */
    private final Trigger mechRT = new Trigger(() -> mech.getRawAxis(rightTrigger) > .5);
    /** Mech LT */
    private final Trigger mechLT = new Trigger(() -> mech.getRawAxis(leftTrigger) > .5);
    /** Mech Up */
    private final Trigger elevatorUp = new Trigger(() -> mech.getPOV() == 0);
    /** Mech Down */
    private final Trigger elevatorDown = new Trigger(() -> mech.getPOV() == 180);
    /** Mech Right */
    private final Trigger resetR = new Trigger(() -> mech.getPOV() == 90);
    /** Mech Left */
    private final Trigger mechLeft = new Trigger(() -> mech.getPOV() == 270);
    /** Mech Start */
    private final JoystickButton resetOdomToPodium = new JoystickButton(mech, XboxController.Button.kStart.value);
    /** Mech Back */
    private final JoystickButton zeroShooter = new JoystickButton(mech, XboxController.Button.kBack.value);
    /** Mech RS */
    private final JoystickButton mechRightStick = new JoystickButton(mech, XboxController.Button.kRightStick.value);
    /** Mech LS */
    private final JoystickButton mechLeftStick = new JoystickButton(mech, XboxController.Button.kLeftStick.value);
// spotless:on
    private final Trigger dynamicForward = new Trigger(() -> FF.getPOV() == 90);
    private final Trigger dynamicBackward = new Trigger(() -> FF.getPOV() == 270);
    private final Trigger quasistaticForward = new Trigger(() -> FF.getPOV() == 0);
    private final Trigger quasistaticBackwards = new Trigger(() -> FF.getPOV() == 180);

    /* Subsystems */
    private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
    private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(m_VisionSubsystem);
    private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    private final IntexerSubsystem m_IntexerSubsystem = new IntexerSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem =
            new ShooterSubsystem(m_SwerveSubsystem, m_ElevatorSubsystem, mech);
    private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem(
            m_VisionSubsystem, m_ShooterSubsystem, m_IntexerSubsystem, m_SwerveSubsystem);

    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // spotless:off
        // Named commands for PathPlanner autos
        NamedCommands.registerCommand("Intake", new IntexForAutosByAutos(m_IntexerSubsystem, m_ShooterSubsystem));
        NamedCommands.registerCommand("Shoot", new AimBot(m_ShooterSubsystem, m_SwerveSubsystem, m_IntexerSubsystem, FiringSolutionsV3.convertToRPM(m_ShooterSubsystem.getCalculatedVelocity())));
        NamedCommands.registerCommand("Subwoofer Shoot", new AimBotSetPosition(m_ShooterSubsystem, m_SwerveSubsystem, m_IntexerSubsystem, FiringSolutionsV3.convertToRPM(m_ShooterSubsystem.getCalculatedVelocity()), Math.toRadians(60)));
        NamedCommands.registerCommand("Idle Speed", new InstantCommand(() -> m_ShooterSubsystem.setShooterVelocity(Constants.Shooter.idleSpeedRPM)));
        NamedCommands.registerCommand("Target Speed", new InstantCommand(() -> m_ShooterSubsystem.setShooterVelocity(FiringSolutionsV3.convertToRPM(m_ShooterSubsystem.getCalculatedVelocity()))));
        NamedCommands.registerCommand("Set Shooter Intake", new InstantCommand(() -> m_IntexerSubsystem.setShooterIntake(.9)));
        NamedCommands.registerCommand("Stop Shooter Intake", new InstantCommand(() -> m_IntexerSubsystem.setShooterIntake(0)));
        NamedCommands.registerCommand("Note Sniffer", new NoteSniffer(m_SwerveSubsystem, m_VisionSubsystem, m_IntexerSubsystem, m_ShooterSubsystem));
        NamedCommands.registerCommand("Note Sniffer2", new NoteSniffer(m_SwerveSubsystem, m_VisionSubsystem, m_IntexerSubsystem, m_ShooterSubsystem));
        NamedCommands.registerCommand("Reset Note", new ResetNoteInShooterPart2(m_ShooterSubsystem, m_IntexerSubsystem, FF));
        NamedCommands.registerCommand("Fire Under Stage", new InstantCommand(() -> m_ShooterSubsystem.setWristByAngle(Math.toRadians(10))));
        NamedCommands.registerCommand("Force Shoot", new FIREEFORACERTAINAMOUNTOFTIME(m_ShooterSubsystem, m_IntexerSubsystem, .2));
        // spotless:on

        m_SwerveSubsystem.setDefaultCommand(new TeleopSwerve(
                m_SwerveSubsystem,
                m_VisionSubsystem,
                m_ShooterSubsystem,
                m_IntexerSubsystem,
                () -> -driver.getRawAxis(leftVerticalAxis),
                () -> -driver.getRawAxis(leftHorizontalAxis),
                () -> -driver.getRawAxis(rightHorizontalAxis),
                () -> false,
                () -> targetAmp.getAsBoolean(),
                () -> targetSpeaker.getAsBoolean(),
                () -> intex.getAsBoolean(),
                () -> intakeNoMove.getAsBoolean(),
                driver));

        m_ElevatorSubsystem.setDefaultCommand(
                new ElevationManual(m_ElevatorSubsystem, () -> -mech.getRawAxis(leftVerticalAxis)));

        m_ShooterSubsystem.setDefaultCommand(new ManRizzt(
                m_ShooterSubsystem,
                () -> -mech.getRawAxis(rightVerticalAxis),
                () -> shooterToSubwoofer.getAsBoolean()));

        autoChooser = AutoBuilder.buildAutoChooser("Shelton Shuffles Back");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* DRIVER BUTTONS */

        // Lock on to speaker
        targetSpeaker.whileTrue(new MissileLock(m_ShooterSubsystem, "speaker"));
        primeShooterSpeedSpeaker
                .negate()
                .and(targetSpeaker)
                .onFalse(new InstantCommand(() ->
                        m_ShooterSubsystem.setShooterVelocity(Constants.Shooter.idleSpeedRPM)));

        targetAmp
                .whileTrue(new MissileLock(m_ShooterSubsystem, "amp"))
                .onFalse(new InstantCommand(() ->
                        m_ShooterSubsystem.setShooterVelocity(Constants.Shooter.idleSpeedRPM)));

        // Shooter
        targetSpeaker
                .or(targetAmp)
                .and(Shoot)
                .whileTrue(new FIREEE(m_ShooterSubsystem, m_IntexerSubsystem)); // Main fire

        // Reset Odometry
        resetOdom.onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroHeading())
                .alongWith(new InstantCommand(() -> m_SwerveSubsystem.setPose(
                        Robot.getAlliance()
                                ? Constants.Vision.startingPoseRed
                                : Constants.Vision.startingPoseBlue))));

        // Intexer
        intex.or(intakeNoMove).whileTrue(new IntexBestHex(m_IntexerSubsystem, true, driver))
         .onFalse(new ResetNoteInShooterPart2(
                m_ShooterSubsystem, m_IntexerSubsystem, driver));
        outex.whileTrue(new IntexBestHex(m_IntexerSubsystem, false, driver));

        // Shooter intake
        forceShoot
                .whileTrue(new InstantCommand(() -> m_IntexerSubsystem.setShooterIntake(Constants.Shooter.shooterOutakeSpeed)))
                .onFalse(new InstantCommand(() -> m_IntexerSubsystem.setShooterIntake(0)));

        // Move to Center Stage
        driverUp.whileTrue(m_SwerveSubsystem.pathToMidfieldChain());
        driverDown.whileTrue(m_SwerveSubsystem.pathToAmp());

        // Move to Source
        // driverDown.whileTrue(m_SwerveSubsystem.pathToSourceChain());

        // Intake from Source
        intakeFromSource
                .whileTrue(new IntakeThroughShooter(
                        m_ShooterSubsystem, m_IntexerSubsystem, m_LEDSubsystem, driver))
                .onFalse(new IntakeThroughShooterPart2(
                        m_ShooterSubsystem, m_IntexerSubsystem, driver));

        /* MECH BUTTONS */

        // Prime Shooter
        primeShooterSpeedSpeaker
                .whileTrue(new InstantCommand(
                        () -> m_ShooterSubsystem.setShooterVelocity(FiringSolutionsV3.convertToRPM(
                                m_ShooterSubsystem.getCalculatedVelocity()))))
                .onFalse(new InstantCommand(() ->
                        m_ShooterSubsystem.setShooterVelocity(Constants.Shooter.idleSpeedRPM)));

        // elevatorDown.onTrue(new ElevatorSet(m_ElevatorSubsystem,
        // Constants.Elevator.minHeightMeters)
        //        .alongWith(
        //              new RizzLevel(m_ShooterSubsystem, Constants.Shooter.intakeAngleRadians)));
        // elevatorUp.onTrue(new ElevatorSet(m_ElevatorSubsystem,
        // Constants.Elevator.maxHeightMeters)
        //        .alongWith(new RizzLevel(m_ShooterSubsystem, 0.0)));

        // Elevator
        mechLT.negate()
                .and(elevatorDown)
                .onTrue(new ElevatorSet(m_ElevatorSubsystem, Constants.Elevator.minHeightMeters)
                        .alongWith(new RizzLevel(
                                m_ShooterSubsystem, Constants.Shooter.intakeAngleRadians)));

        mechLT.negate()
                .and(elevatorUp)
                .onTrue(new ElevatorSet(m_ElevatorSubsystem, Constants.Elevator.maxHeightMeters)
                        .alongWith(new RizzLevel(m_ShooterSubsystem, 0.0)));

        // Adjust offset
        mechLT.and(elevatorUp).onTrue(new InstantCommand(() -> m_ShooterSubsystem.offsetUP()));
        mechLT.and(elevatorDown).onTrue(new InstantCommand(() -> m_ShooterSubsystem.offsetDOWN()));

        // Zero wrist
        mechLT.negate()
                .and(zeroShooter)
                .onTrue(new InstantCommand(() -> m_ShooterSubsystem.resetWristEncoders(
                        Constants.Shooter.angleOffsetTop))); // Set encoder to zero

        // Reset Odom to Podium
        resetOdomToPodium.onTrue(new InstantCommand(() -> m_SwerveSubsystem.setPoseToPodium()));

        // Intake Preset
        shooterToIntake
                .onTrue(new RizzLevel(m_ShooterSubsystem, Constants.Shooter.intakeAngleRadians))
                .onTrue(new ElevatorSet(m_ElevatorSubsystem, Constants.Elevator.minHeightMeters));

        // Amp Preset
        shooterToAmp
                .onTrue(new RizzLevel(m_ShooterSubsystem, -0.40))
                .onTrue(new ElevatorSet(m_ElevatorSubsystem, Constants.Elevator.ampHeight));

        // Subwoofer Preset
        shooterToSubwoofer
                .onTrue(new RizzLevel(m_ShooterSubsystem, Math.toRadians(60)))
                .onTrue(new ElevatorSet(m_ElevatorSubsystem, Constants.Elevator.minHeightMeters));

        // Anti-Defense Preset
        shooterToAntiDefense.onTrue(new ElevatorSet(
                m_ElevatorSubsystem,
                Constants.Elevator.antiBozoSmileToasterAhhGoonBotShooterHeight));

        // Reset the R calculation in case it gets off
        resetR.onTrue(new InstantCommand(() -> FiringSolutionsV3.resetAllR()));

        // Reset Note in Shooter
        resetNoteInShooter
                .whileTrue(new ResetNoteInShooter(m_ShooterSubsystem, m_IntexerSubsystem, mech))
                .onFalse(new ResetNoteInShooterPart2(m_ShooterSubsystem, m_IntexerSubsystem, mech));

        // Kill Flywheels
        mechLT.negate()
                .and(mechRightStick)
                .onTrue(new InstantCommand(() -> m_ShooterSubsystem.setShooterVelocity(0)));

        // Kill Wrist
        mechLT.and(mechRightStick)
                .onTrue(new InstantCommand(() -> m_ShooterSubsystem.setWristSpeedManual(0))
                        .alongWith(new InstantCommand(() -> m_ShooterSubsystem.setWristToCoast())))
                .onFalse(new InstantCommand(() -> m_ShooterSubsystem.setWristToBrake()));

        // Kill Elevator
        mechLT.and(mechLeftStick)
                .onTrue(new InstantCommand(() -> m_ElevatorSubsystem.setElevatorSpeedManual(0)));

        // Reset wrist from bottom
        mechLT.and(zeroShooter)
                .onTrue(new InstantCommand(() -> m_ShooterSubsystem.resetWristEncoders(
                        Constants.Shooter.angleOffsetBottom)));

        /* THIRD CONTROLLER */
        // Characterization tests
        
        dynamicForward.whileTrue(m_SwerveSubsystem.sysIdDynamic(Direction.kForward));
        dynamicBackward.whileTrue(m_SwerveSubsystem.sysIdDynamic(Direction.kReverse));
        quasistaticForward.whileTrue(m_SwerveSubsystem.sysIdQuasistatic(Direction.kForward));
        quasistaticBackwards.whileTrue(m_SwerveSubsystem.sysIdQuasistatic(Direction.kReverse));
        
    }

    public void stopAll() {
        m_ShooterSubsystem.setShooterVelocity(0);
        m_ShooterSubsystem.setWristSpeedManual(0);
        m_IntexerSubsystem.setALL(0);
        m_ElevatorSubsystem.setPositionWithEncoder(m_ElevatorSubsystem.getPosition());
    }

    public void zeroWristEncoders() {
        m_ShooterSubsystem.resetWristEncoders(Constants.Shooter.angleOffsetTop);
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
