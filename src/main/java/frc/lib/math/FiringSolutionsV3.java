package frc.lib.math;
// spotless:off
import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;

public class FiringSolutionsV3 {

    // Robot Values
    private static final double defaultShooterHeight = 0.42672;
    private static double shooterHeight = 0.42672;
    private static final double laserCANOffset = 0;
    public static final double maxShooterAngle = Math.toRadians(70);
    public static double slipPercent = 0.5324;
    private static double shooterVelocity = 10.0;
    private static final double noteFallAccel = 9.8;

    // Amp Values (used for targeting from afar)
    private static final double ampTargetXRed = 15.91;
    private static final double ampTargetXBlue = 16.54 - ampTargetXRed;
    public static double ampTargetX;
    public static final double ampTargetY = 7;
    public static final double ampTargetZ = .125;

    // True Amp Values
    private static final double trueAmpXBlue = 1.8415;
    private static final double trueAmpXRed = 16.54 - trueAmpXBlue;
    public static double trueAmpX;
    public static final double trueAmpY = 8.2;

    // Speaker Values
    public static double speakerTargetXOffset = .24;
    private static double speakerTargetXBlue = 0.0 + speakerTargetXOffset;
    private static double speakerTargetXRed = 16.54 - speakerTargetXOffset;
    public static double speakerTargetX;
    public static final double speakerTargetY = 5.55;
    public static double speakerTargetZ = 1.92;

    //R Values
    private static double speakerR, ampR, customR = 1.0;
    public static double maxRangeWithR = 11.25;

    private FiringSolutionsV3() {
    }

    public static void setAlliance(boolean redAlliance) {
        if (redAlliance) {
            speakerTargetX = speakerTargetXRed;
            ampTargetX = ampTargetXRed;
            trueAmpX = trueAmpXRed;
        } else {
            speakerTargetX = speakerTargetXBlue;
            ampTargetX = ampTargetXBlue;
            trueAmpX = trueAmpXBlue;
        }
    }

    public static void updateHeight(double laserCANValue) {
        shooterHeight = defaultShooterHeight + (laserCANValue - laserCANOffset);
    }

    public static double getAngleToTarget(double robotX, double robotY, double targetX, double targetY) {
        return Math.atan((targetY - robotY) / (targetX - robotX));
    }

    public static double getDistanceToTarget(double robotX, double robotY, double targetX, double targetY) {
        return Math.abs(Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2)));
    }

    public static double getRobotVelocityTowardsTarget(double robotX, double targetX, double robotVelocityX,
            double robotVelocityY,
            double angleToTarget, double robotHeading) {
        if (robotX <= targetX) {
            if (robotVelocityX == 0) {
                return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2))
                        * Math.cos(-angleToTarget - robotHeading);
            } else {
                return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2))
                        * Math.cos(Math.atan(robotVelocityY / robotVelocityX) - angleToTarget - robotHeading);
            }
        } else {
            if (robotVelocityX == 0) {
                return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2))
                        * Math.cos(angleToTarget + robotHeading);
            } else {
                return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2))
                        * Math.cos(Math.atan(robotVelocityY / robotVelocityX) + angleToTarget + robotHeading);
            }
        }
    }

    public static double getRobotVelocityPerpendicularToTarget(double robotX, double targetX, double robotVelocityX,
            double robotVelocityY,
            double angleToSpeaker, double robotHeading) {
        if (robotX <= targetX) {
            if (robotVelocityX == 0) {
                return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2))
                        * Math.sin(-angleToSpeaker - robotHeading);
            } else {
                return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2))
                        * Math.sin(Math.atan(robotVelocityY / robotVelocityX) - angleToSpeaker - robotHeading);
            }
        } else {
            if (robotVelocityX == 0) {
                return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2))
                        * Math.sin(angleToSpeaker + robotHeading);
            } else {
                return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2))
                        * Math.sin(Math.atan(robotVelocityY / robotVelocityX) + angleToSpeaker + robotHeading);
            }
        }
    }

    public static double C(double distanceToTarget) {
        return noteFallAccel * Math.pow(distanceToTarget, 2);
    }

    public static double quarticA(double distanceToTarget, double targetZ) {
        return 4 * Math.pow(shooterVelocity, 4)
                * (Math.pow(targetZ - shooterHeight, 2) + Math.pow(distanceToTarget, 2));
    }

    public static double quarticB() {
        return 0;
    }

    public static double quarticC(double distanceToTarget, double targetZ) {
        return 4 * Math.pow(shooterVelocity, 2)
                * (C(distanceToTarget) * (targetZ - shooterHeight)
                        - Math.pow(shooterVelocity, 2) * Math.pow(distanceToTarget, 2));
    }

    public static double quarticD() {
        return 0;
    }

    public static double quarticE(double distanceToTarget) {
        return Math.pow(C(distanceToTarget), 2);
    }

    public static void updateSpeakerR(double distanceToSpeaker) {
        speakerR = speakerR - ((quarticA(distanceToSpeaker, speakerTargetZ) * Math.pow(speakerR, 4)
                + quarticC(distanceToSpeaker, speakerTargetZ) * Math.pow(speakerR, 2)
                + quarticE(distanceToSpeaker))
                / (4 * quarticA(distanceToSpeaker, speakerTargetZ) * Math.pow(speakerR, 3)
                        + 2 * quarticC(distanceToSpeaker, speakerTargetZ) * speakerR));
    }

    public static void resetSpeakerR() {
        speakerR = 1.0;
    }

    public static double getSpeakerR() {
        return speakerR;
    }

    public static double getShooterAngleFromSpeakerR() {
        return Math.acos(speakerR);
    }

    public static void updateAmpR(double distanceToAmp) {
        ampR = ampR - ((quarticA(distanceToAmp, speakerTargetZ) * Math.pow(ampR, 4)
                + quarticC(distanceToAmp, speakerTargetZ) * Math.pow(ampR, 2)
                + quarticE(distanceToAmp))
                / (4 * quarticA(distanceToAmp, speakerTargetZ) * Math.pow(ampR, 3)
                        + 2 * quarticC(distanceToAmp, speakerTargetZ) * ampR));
    }

    public static void resetAmpR() {
        ampR = 1.0;
    }

    public static double getAmpR() {
        return ampR;
    }

    public static double getShooterAngleFromAmpR() {
        return Math.acos(ampR);
    }

    public static void updateCustomR(double distanceToTarget, double targetZ) {
        customR = customR - ((quarticA(distanceToTarget, targetZ) * Math.pow(customR, 4)
                + quarticC(distanceToTarget, targetZ) * Math.pow(customR, 2)
                + quarticE(distanceToTarget))
                / (4 * quarticA(distanceToTarget, targetZ) * Math.pow(customR, 3)
                        + 2 * quarticC(distanceToTarget, targetZ) * customR));
    }

    public static void resetCustomR() {
        customR = 1.0;
    }

    public static double getCustomR() {
        return customR;
    }

    public static double getShooterAngleFromCustomR() {
        return Math.acos(customR);
    }

    public static void resetAllR() {
        resetSpeakerR();
        resetAmpR();
        resetCustomR();
    }

    public static double smallestWhichIsntNegativeOrNan(double t1, double t2) {
        if (t1 >= 0 && t2 < 0) {
            return t1;
        } else if (t2 >= 0 && t1 < 0) {
            return t2;
        } else if (t1 > 0 && t2 > 0) {
            if (t1 <= t2) {
                return t1;
            } else {
                return t2;
            }
        } else {
            return 0;
        }
    }

    private static double pow(double a) {
        return Math.pow(a, 2);
    }

    private static boolean check(double a) {
        return Double.isFinite(a) && a > 0;
    }

    public static Optional<Translation2d> movingTarget(
            double robotX,
            double robotY,
            double targetX,
            double targetY,
            double robotVelocityTowardsSpeaker,
            double robotVelocityPerpendicularToSpeaker) {
        Translation2d robotPos = new Translation2d(robotX, robotY); // P1
        Translation2d targetVel = new Translation2d(-robotVelocityTowardsSpeaker, -robotVelocityPerpendicularToSpeaker); // V0
        Translation2d targetPos = new Translation2d(targetX, targetY); // P0

        // uses code from: https://stackoverflow.com/a/22117046/21621189, look in
        // comments for errors and edge cases accounted for

        // a desmos visualization: https://www.desmos.com/calculator/ejg6jrsodq

        double a = pow(targetVel.getX()) + pow(targetVel.getY()) - pow(shooterVelocity);
        double b = 2 * ((targetPos.getX() * targetVel.getX()) + (targetPos.getY()
                * targetVel.getY()) - (robotPos.getX() * targetVel.getX()) - (robotPos.getY() * targetVel.getY()));
        double c = pow(targetPos.getX()) + pow(targetPos.getY())
                + pow(robotPos.getX()) + pow(robotPos.getY())
                - (2 * robotPos.getX() * targetPos.getX()) - (2 * robotPos.getY() * targetPos.getY());

        if (a == 0)
            return Optional.empty();

        double t1 = (-b + Math.sqrt((b * b) - (4 * a * c))) / (2 * a);
        double t2 = (-b - Math.sqrt((b * b) - (4 * a * c))) / (2 * a);

        double t; // t = time at collision (from 0) in time unit of speakVel

        if (!check(t1) && !check(t2)) {
            return Optional.empty();
        } else if (!check(t1) || !check(t2)) {
            t = check(t1) ? t1 : t2;
        } else {
            t = Math.min(t1, t2);
        }

        Translation2d out = targetPos.plus(targetVel.times(t));

        return Optional.of(out); // outputs the speakerPos at the collision time relative to the robot, (rotation
                                 // is based on the same as the input translate2ds)
    }

    public static double getAngleToMovingTarget(double robotX, double robotY, double targetX, double targetY,
            double robotVelocityX,
            double robotVelocityY, double robotHeading) {
        //if (robotX >= targetX) {
            return Math.atan((movingTarget(robotX, robotY, targetX, targetY,
                    getRobotVelocityTowardsTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                            getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading),
                    getRobotVelocityPerpendicularToTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                            getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading))
                    .get().getY() - robotY)
                    / (movingTarget(robotX, robotY, targetX, targetY,
                            getRobotVelocityTowardsTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                                    getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading),
                            getRobotVelocityPerpendicularToTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                                    getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading))
                            .get().getX() - robotX));
        /*} else {
            if (robotY >= targetY) {
                return Math.atan((movingTarget(robotX, robotY, targetX, targetY,
                        getRobotVelocityTowardsTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                                getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading),
                        getRobotVelocityPerpendicularToTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                                getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading))
                        .get().getY() - robotY)
                        / (movingTarget(robotX, robotY, targetX, targetY,
                                getRobotVelocityTowardsTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                                        getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading),
                                getRobotVelocityPerpendicularToTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                                        getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading))
                                .get().getX() - robotX)) + Math.toRadians(180);
            } else {
                return Math.atan((movingTarget(robotX, robotY, targetX, targetY,
                        getRobotVelocityTowardsTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                                getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading),
                        getRobotVelocityPerpendicularToTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                                getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading))
                        .get().getY() - robotY)
                        / (movingTarget(robotX, robotY, targetX, targetY,
                                getRobotVelocityTowardsTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                                        getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading),
                                getRobotVelocityPerpendicularToTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                                        getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading))
                                .get().getX() - robotX)) - Math.toRadians(180);
            }
        }*/

    }

    public static double getDistanceToMovingTarget(double robotX, double robotY, double targetX, double targetY,
            double robotVelocityX,
            double robotVelocityY, double robotHeading) {
        return Math.abs(Math.sqrt(
                Math.pow(movingTarget(robotX, robotY, targetX, targetY,
                        getRobotVelocityTowardsTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                                getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading),
                        getRobotVelocityPerpendicularToTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                                getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading))
                        .get().getX() - robotX, 2)
                        + Math.pow(
                                movingTarget(robotX, robotY, targetX, targetY,
                                        getRobotVelocityTowardsTarget(robotX, targetX, robotVelocityX, robotVelocityY,
                                                getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading),
                                        getRobotVelocityPerpendicularToTarget(robotX, targetX, robotVelocityX,
                                                robotVelocityY,
                                                getAngleToTarget(robotX, robotY, targetX, targetY), robotHeading))
                                        .get().getY() - robotY,
                                2)));
    }

    /** Convert meters per second to rotations per minute */
    public static double convertToRPM(double velocity) {
        return (60 * velocity) / (slipPercent * Math.PI * .1016);
    }
// spotless:on
}
