// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Interpolations {

  public InterpolatingDoubleTreeMap shooterSpeeds = new InterpolatingDoubleTreeMap();

  public Interpolations() {
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
  }

  public double getShooterAngleFromInterpolation(double distanceToTarget){
    return shooterSpeeds.get(distanceToTarget);
  }
}