// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;

public class LEDSubsystem extends SubsystemBase {
    public DigitalOutput WalterLight = new DigitalOutput(3); // Bit 1 (1)
    public DigitalOutput JesseBlinkman = new DigitalOutput(4); // Bit 2 (2)
    public DigitalOutput GusBling = new DigitalOutput(5); // Bit 3 (4)
    public DigitalOutput SkylerBright = new DigitalOutput(6); // Bit 4 (8)

    // Output bits to the LEDs
    public DigitalOutput[] bits = {WalterLight, JesseBlinkman, GusBling, SkylerBright
    }; // Actual Outputs
    private boolean[] output = new boolean[4]; // Computed Outputs

    // Booleans used for Easy input
    private Boolean hasNote = false;
    private Boolean chargingOuttake = false;
    private Boolean atSpeed = false;

    public Boolean[] inputBooleans = {
        false, // Disconnected -0 Loading Bounce
        false, // Pathfinding -1 Rainbow Pattern #1
        false, // Climb -2 Rainbow Pattern #2
        false, // Blank -3 None
        false, // Note Detected -4 White Solid
        false, // Charging -5 Green Pulse (HasNote)
        false, // At Speed -6 Green BLink (HasNote)
        false, // Charging -7 Magenta Pulse (NoNote)
        false, // At Speed -8 Magenta BLink (NoNote)
        false, // Note in Intake -9 Orange Blink
        false, // Note in Shooter -10 Orange Solid
        false, // Alliance Color -11 Red Pulse
        false // Alliance Color -12 Blue Pulse
    };

    public int[] priorities = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12
    }; // 'priorities' values are the indexes of the list inputBooleans; Priority: Left to Right

    // Use subsystems
    VisionSubsystem vision;
    ShooterSubsystem shooter;
    IntexerSubsystem intexer;
    SwerveSubsystem swerve;

    /** Creates a new LEDSubsystem. */
    public LEDSubsystem(
            VisionSubsystem m_VisionSubsystem,
            ShooterSubsystem m_ShooterSubsystem,
            IntexerSubsystem m_IntexerSubsystem,
            SwerveSubsystem m_SwerveSubsystem) {
        this.vision = m_VisionSubsystem;
        this.shooter = m_ShooterSubsystem;
        this.intexer = m_IntexerSubsystem;
        this.swerve = m_SwerveSubsystem;
    }

    @Override
    public void periodic() {
        set(); // Decimal Phase
        encoder(); // Transition Phase
        send(); // Send Phase
    }

    private void set() { // Decimal phase
        var results = vision.getLatestResultN();

        // Note Detected
        if (results.hasTargets()) {
            NoteDetected(true);
        } else {
            NoteDetected(false);
        }

        // Driver Station Connected
        if (DriverStation.isDSAttached()) {
            Disconnected(false);
        } else {
            Disconnected(true);
        }

        if (Robot.checkRedAlliance()) {
            RedAlliance(true);
            BlueAlliance(false);
        } else {
            RedAlliance(false);
            BlueAlliance(true);
        }

        // Check if pathfinding
        if (SwerveSubsystem.followingPath) {
            Pathfinding(true);
        } else {
            Pathfinding(false);
        }

        // Check beam breaks
        if (intexer.intakeBreak()) {
            NoteInIntake(true); // Intake
            NoteInShooter(false); // Shooter
            hasNote = true;
        } else if (intexer.shooterBreak()) {
            NoteInIntake(false); // Intake
            NoteInShooter(true); // Shooter
            hasNote = true;
        } else {
            NoteInIntake(false); // Intake
            NoteInShooter(false); // Shooter
            hasNote = false;
        }

        // Charging or At Speed with Note or without Note !Boolean References See
        // Above(inputBooleans)!
        if (hasNote) { // Has Note
            if (shooter.isShooterAtSpeed()) { // At speed
                inputBooleans[5] = false;
                inputBooleans[6] = true;
            } else if (shooter.getVelocity() > Constants.Shooter.idleSpeedRPM + 500) { // Charging
                inputBooleans[5] = true;
                inputBooleans[6] = false;
            } else {
                inputBooleans[5] = false;
                inputBooleans[6] = false;
            }
            inputBooleans[7] = false;
            inputBooleans[8] = false;
        } else { // No Note
            if (shooter.isShooterAtSpeed()) { // At speed
                inputBooleans[7] = false;
                inputBooleans[8] = true;
            } else if (shooter.getVelocity() > Constants.Shooter.idleSpeedRPM + 500) { // Charging
                inputBooleans[7] = true;
                inputBooleans[8] = false;
            } else {
                inputBooleans[7] = false;
                inputBooleans[8] = false;
            }
            inputBooleans[5] = false;
            inputBooleans[6] = false;
        }

        SmartDashboard.putBooleanArray("Input Booleans", inputBooleans);
    }

    private void encoder() { // Transition phase
        // Decimal to Binary
        int pin_amount = 4; // Amount of pins

        int trueIndex = 0; // Index of selected LED sequence

        for (int i = 0;
                i < priorities.length;
                i++) { // Picks the first true sequence based on priority
            if (inputBooleans[priorities[i]]) {
                trueIndex = priorities[i];
                break;
            }
        }

        String binaryString = Integer.toBinaryString(trueIndex); // Cast Number to Binary
        int length = binaryString.length(); // Find length of Binary (How many bits)
        String finalString; // Final binary string

        // Adds the amount of 0's needed for acurate transcription. Ex. 0x01 -> 0x0001
        if (length < pin_amount) {
            int zerosToAdd = pin_amount - length;
            StringBuilder paddedStringBuilder = new StringBuilder();

            for (int i = 0; i < zerosToAdd; i++) {
                paddedStringBuilder.append("0");
            }

            paddedStringBuilder.append(binaryString);
            String paddedBinaryString = paddedStringBuilder.toString();

            finalString = paddedBinaryString;
        } else {
            finalString = binaryString;
        }

        SmartDashboard.putString("Binary Output", finalString);

        for (int i = pin_amount - 1; i >= 0; i--) {
            output[i] = finalString.charAt(i) == '1';
        }
    }

    private void send() { // Send Phase - "Redundancy is bliss"
        for (int i = 0; i < output.length; i++) {
            bits[i].set(output[i]);
        }
        SmartDashboard.putBooleanArray("DigitalOutputs", output);
    }

    // SET FUNCTIONS
    public void Pathfinding(boolean pathfinding) {
        inputBooleans[1] = pathfinding;
    }

    public void climbTele(boolean climb) {
        inputBooleans[2] = climb;
    }

    public void HasNote(boolean hasNote) {
        this.hasNote = hasNote;
    }

    public void ChargingOuttake(boolean chargingOuttake) {
        this.chargingOuttake = chargingOuttake;
    }

    public void ReadyToFire(boolean fire) {
        this.atSpeed = fire;
    }

    public void NoteInIntake(boolean noteInIntake) {
        inputBooleans[9] = noteInIntake;
    }

    public void NoteInShooter(boolean noteInShooter) {
        inputBooleans[10] = noteInShooter;
    }

    public void NoteDetected(boolean note) {
        inputBooleans[4] = note;
    }

    public void BlueAlliance(boolean blueAlliance) {
        inputBooleans[12] = blueAlliance;
    }

    public void RedAlliance(boolean redAlliance) {
        inputBooleans[13] = redAlliance;
    }

    public void Disconnected(boolean disconnected) { // NOT CONFIRMED TO BE IN FINAL
        inputBooleans[0] = disconnected;
    }
}
