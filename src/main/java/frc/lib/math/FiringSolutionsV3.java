package frc.lib.math;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;

public class FiringSolutionsV3 {

    private static final double shooterHeight = 0.371;
    private static final double noteFallAccel = 9.8;
    private static final double shooterTargetXBlue = 0.24;
    private static final double shooterTargetXRed = 16.3;
    private static final double shooterTargetY = 5.55;
    private static final double slipPercent = .67;
    private static final double maxShooterAngle = Math.toRadians(70);
    public static double shooterTargetZ = 1.98;

    private static double shooterTargetX;
    private static double shooterVelocity = 10.0;
    private static double R = 1.0;

    private FiringSolutionsV3() {
    }

    public static void setAlliance(boolean redAlliance) {
        if (redAlliance) {
            shooterTargetX = shooterTargetXRed;
        } else {
            shooterTargetX = shooterTargetXBlue;
        }
    }

    public static double getAngleToSpeaker(double robotX, double robotY) {
        return Math.atan((shooterTargetY - robotY) / (shooterTargetX - robotX));
    }

    public static double getDistanceToSpeaker(double robotX, double robotY) {
        return Math.abs(Math.sqrt(Math.pow(shooterTargetX - robotX, 2) + Math.pow(shooterTargetY - robotY, 2)));
    }

    public static double getRobotVelocityTowardsSpeaker(double robotVelocityX, double robotVelocityY,
            double angleToSpeaker, double robotHeading) {
        if (shooterTargetX == shooterTargetXRed) {
            if (robotVelocityX == 0) {
                return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2))
                        * Math.cos(-angleToSpeaker - robotHeading);
            } else {
                return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2))
                        * Math.cos(Math.atan(robotVelocityY / robotVelocityX) - angleToSpeaker - robotHeading);
            }
        } else {
            if (robotVelocityX == 0) {
                return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2))
                        * Math.cos(angleToSpeaker + robotHeading);
            } else {
                return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2))
                        * Math.cos(Math.atan(robotVelocityY / robotVelocityX) + angleToSpeaker + robotHeading);
            }
        }
    }

    public static double getRobotVelocityPerpendicularToSpeaker(double robotVelocityX, double robotVelocityY,
            double angleToSpeaker, double robotHeading) {
        if (shooterTargetX == shooterTargetXRed) {
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

    public static double C(double distanceToSpeaker) {
        return noteFallAccel * Math.pow(distanceToSpeaker, 2);
    }

    public static double quarticA(double distanceToSpeaker) {
        return 4 * Math.pow(shooterVelocity, 4)
                * (Math.pow(shooterTargetZ - shooterHeight, 2) + Math.pow(distanceToSpeaker, 2));
    }

    public static double quarticB() {
        return 0;
    }

    public static double quarticC(double distanceToSpeaker) {
        return 4 * Math.pow(shooterVelocity, 2)
                * (C(distanceToSpeaker) * (shooterTargetZ - shooterHeight)
                        - Math.pow(shooterVelocity, 2) * Math.pow(distanceToSpeaker, 2));
    }

    public static double quarticD() {
        return 0;
    }

    public static double quarticE(double distanceToSpeaker) {
        return Math.pow(C(distanceToSpeaker), 2);
    }

    public static void updateR(double distanceToSpeaker) {
        R = R - ((quarticA(distanceToSpeaker) * Math.pow(R, 4)
                + quarticC(distanceToSpeaker) * Math.pow(R, 2)
                + quarticE(distanceToSpeaker))
                / (4 * quarticA(distanceToSpeaker) * Math.pow(R, 3)
                        + 2 * quarticC(distanceToSpeaker) * R));

        /*if (R < 0) {
            resetR();
        }*/

    }

    public static void resetR() {
        R = 1.0;
    }

    public static double getR() {
        return R;
    }

    public static double getShooterAngle() {
        return Math.acos(R);
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
            double robotVelocityTowardsSpeaker,
            double robotVelocityPerpendicularToSpeaker) {
        Translation2d robotPos = new Translation2d(robotX, robotY); // P1
        Translation2d speakPos = new Translation2d(shooterTargetX, shooterTargetY); // P0
        Translation2d speakVel = new Translation2d(-robotVelocityTowardsSpeaker, -robotVelocityPerpendicularToSpeaker); // V0

        // uses code from: https://stackoverflow.com/a/22117046/21621189, look in
        // comments for errors and edge cases accounted for

        // a desmos visualization: https://www.desmos.com/calculator/ejg6jrsodq

        double a = pow(speakVel.getX()) + pow(speakVel.getY()) - pow(shooterVelocity);
        double b = 2 * ((speakPos.getX() * speakVel.getX()) + (speakPos.getY()
                * speakVel.getY()) - (robotPos.getX() * speakVel.getX()) - (robotPos.getY() * speakVel.getY()));
        double c = pow(speakPos.getX()) + pow(speakPos.getY())
                + pow(robotPos.getX()) + pow(robotPos.getY())
                - (2 * robotPos.getX() * speakPos.getX()) - (2 * robotPos.getY() * speakPos.getY());

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

        Translation2d out = speakPos.plus(speakVel.times(t));

        return Optional.of(out); // outputs the speakerPos at the collision time relative to the robot, (rotation
                                 // is based on the same as the input translate2ds)
    }

    public static double getAngleToMovingTarget(double robotX, double robotY, double robotVelocityX,
            double robotVelocityY, double robotHeading) {
        return Math.atan((movingTarget(robotX, robotY,
                getRobotVelocityTowardsSpeaker(robotVelocityX, robotVelocityY,
                        getAngleToSpeaker(robotX, robotY), robotHeading),
                getRobotVelocityPerpendicularToSpeaker(robotVelocityX, robotVelocityY,
                                                getAngleToSpeaker(robotX, robotY), robotHeading)).get().getY() - robotY)
                / (movingTarget(robotX, robotY,
                        getRobotVelocityTowardsSpeaker(robotVelocityX, robotVelocityY,
                                getAngleToSpeaker(robotX, robotY), robotHeading),
                        getRobotVelocityPerpendicularToSpeaker(robotVelocityX, robotVelocityY,
                                                getAngleToSpeaker(robotX, robotY), robotHeading)).get().getX() - robotX));
    }

    public static double getDistanceToMovingTarget(double robotX, double robotY, double robotVelocityX,
            double robotVelocityY, double robotHeading) {
        return Math.abs(Math.sqrt(
                Math.pow(movingTarget(robotX, robotY,
                        getRobotVelocityTowardsSpeaker(robotVelocityX, robotVelocityY,
                                getAngleToSpeaker(robotX, robotY), robotHeading),
                        getRobotVelocityPerpendicularToSpeaker(robotVelocityX, robotVelocityY,
                                                getAngleToSpeaker(robotX, robotY), robotHeading)).get().getX() - robotX, 2)
                        + Math.pow(
                                movingTarget(robotX, robotY,
                                        getRobotVelocityTowardsSpeaker(robotVelocityX, robotVelocityY,
                                                getAngleToSpeaker(robotX, robotY), robotHeading),
                                        getRobotVelocityPerpendicularToSpeaker(robotVelocityX, robotVelocityY,
                                                getAngleToSpeaker(robotX, robotY), robotHeading)).get().getY() - robotY,
                                2)));
    }

}
