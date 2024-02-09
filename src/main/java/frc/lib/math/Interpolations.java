// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

class InterpolatingDoubleTreeMapTest {

  void ShooterDataInterpolater() {
    InterpolatingDoubleTreeMap shooterSpeeds[] = {};

    shooterSpeeds[10] = new InterpolatingDoubleTreeMap();
    shooterSpeeds[10].put(1.5, 55.0);
    shooterSpeeds[10].put(1.75, 52.4);
    shooterSpeeds[10].put(2.0, 49.0);
    shooterSpeeds[10].put(2.25, 46.0);
    shooterSpeeds[10].put(2.5, 43.5);
    shooterSpeeds[10].put(2.75, 41.6);
    shooterSpeeds[10].put(3.0, 40.0);
    shooterSpeeds[10].put(3.25, 38.5);
    shooterSpeeds[10].put(3.5, 37.5);
    shooterSpeeds[10].put(3.75, 36.6);
    shooterSpeeds[10].put(4.0, 36.0);
    shooterSpeeds[10].put(4.25, 35.5);
    shooterSpeeds[10].put(4.5, 35.2);
    shooterSpeeds[10].put(4.75, 35.0);
    
    shooterSpeeds[11] = new InterpolatingDoubleTreeMap();
    shooterSpeeds[11].put(1.5, 55.0);
    shooterSpeeds[11].put(1.75, 51.8);
    shooterSpeeds[11].put(2.0, 47.7);
    shooterSpeeds[11].put(2.25, 44.8);
    shooterSpeeds[11].put(2.5, 42.0);
    shooterSpeeds[11].put(3.0, 38.1);
    shooterSpeeds[11].put(3.5, 35.5);
    shooterSpeeds[11].put(4.0, 33.5);
    shooterSpeeds[11].put(4.5, 32.4);
    shooterSpeeds[11].put(5.0, 31.7);
    shooterSpeeds[11].put(5.5, 31.4);

    shooterSpeeds[12] = new InterpolatingDoubleTreeMap();
    shooterSpeeds[12].put(1.5, 55.0);
    shooterSpeeds[12].put(1.75,51.2);
    shooterSpeeds[12].put(2.0, 47.0);
    shooterSpeeds[12].put(2.25, 43.9);
    shooterSpeeds[12].put(2.5, 41.0);
    shooterSpeeds[12].put(3.0, 36.8);
    shooterSpeeds[12].put(3.5, 34.0);
    shooterSpeeds[12].put(4.0, 31.8);
    shooterSpeeds[12].put(4.5, 30.4);
    shooterSpeeds[12].put(5.0, 29.4);
    shooterSpeeds[12].put(5.5, 28.9);
    shooterSpeeds[12].put(6.0, 28.6);
    shooterSpeeds[12].put(6.5, 28.4);

    shooterSpeeds[13] = new InterpolatingDoubleTreeMap();
    shooterSpeeds[13].put(1.5, 55.0);
  }
}