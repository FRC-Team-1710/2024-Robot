// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Interpolations {

    public InterpolatingDoubleTreeMap shooterSpeeds = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds =
            new InterpolatingDoubleTreeMap();

    public Interpolations() {
        /*
                shooterSpeeds.put(1.25, 53.0);
                shooterSpeeds.put(1.5, 48.0);
                shooterSpeeds.put(1.75, 44.0);
                shooterSpeeds.put(2.0, 41.0);
                shooterSpeeds.put(2.25, 39.0);
                shooterSpeeds.put(2.5, 37.5);
                shooterSpeeds.put(2.75, 35.5);
                shooterSpeeds.put(3.0, 34.0);
                shooterSpeeds.put(3.25, 33.0);
                shooterSpeeds.put(3.5, 32.0);
                shooterSpeeds.put(4.0, 30.75);
                shooterSpeeds.put(4.5, 29.75);

        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(1.25, 44.0);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(1.5, 39.5);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(1.75, 36.0);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(2.0, 32.25);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(2.25, 30.0);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(2.5, 28.0);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(2.75, 26.5);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(3.0, 24.5);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(3.25, 22.5);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(3.5, 21.25);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(3.75, 20.0);*/

        /*shooterSpeeds.put(1.25, 56.0);
        shooterSpeeds.put(1.35, 53.0);
        shooterSpeeds.put(1.5, 50.0);
        shooterSpeeds.put(1.75, 46.0);
        shooterSpeeds.put(2.0, 42.0);
        shooterSpeeds.put(2.25, 38.5);
        shooterSpeeds.put(2.5, 35.5);
        shooterSpeeds.put(2.75, 33.75);
        shooterSpeeds.put(3.0, 32.25);
        shooterSpeeds.put(3.25, 31.0);
        shooterSpeeds.put(3.5, 30.0);
        shooterSpeeds.put(4.0, 28.25);*/

        /* Seven Rivers */
        shooterSpeeds.put(1.25, 62.0);
        shooterSpeeds.put(1.33, 59.0);
        shooterSpeeds.put(1.5, 56.0);
        shooterSpeeds.put(1.75, 52.5);
        shooterSpeeds.put(2.0, 49.0);
        shooterSpeeds.put(2.25, 46.5);
        shooterSpeeds.put(2.5, 44.5);
        shooterSpeeds.put(2.75, 42.0);
        shooterSpeeds.put(3.0, 39.75);
        shooterSpeeds.put(3.25, 38.0);
        shooterSpeeds.put(3.5, 37.0);
        shooterSpeeds.put(3.75, 36.0);
        shooterSpeeds.put(4.0, 35.0);
        shooterSpeeds.put(4.25, 34.0);
        shooterSpeeds.put(4.5, 33.0);
        shooterSpeeds.put(4.75, 32.66);
        shooterSpeeds.put(5.0, 32.33);

        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(1.25, 50.0);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(1.33, 47.0);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(1.5, 43.0);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(1.75, 39.0);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(2.0, 36.0);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(2.25, 34.0);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(2.5, 32.5);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(2.75, 31.25);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(3.0, 30.0);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(3.25, 29.0);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(3.5, 28.0);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(3.75, 27.25);
        bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.put(4.0, 26.5);
    }

    public double getShooterAngleFromInterpolation(double distanceToTarget) {
        return shooterSpeeds.get(distanceToTarget);
    }

    public double getShooterAngleFromInterpolationElevatorUp(double distanceToTarget) {
        return bozoDefenseBotDriversHonestReactionWhenElevatorGoUp_speeds.get(distanceToTarget);
    }
}
