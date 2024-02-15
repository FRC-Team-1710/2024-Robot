package frc.lib.math;

public class FiringSolutionsV2 {

    private static final double shooterHeight = 0.355;
    private static final double noteFallAccel = 9.8;
    private static final double shooterTargetXBlue = 0.24;
    private static final double shooterTargetXRed = 16.3;
    private static final double shooterTargetY = 5.55;
    private static final double shooterTargetZ = 2.04;
    private static final double slipPercent = .67;

    private static double shooterTargetX;
    private static double shooterVelocity;
    private static double R = 0.0;

    private FiringSolutionsV2() {
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

    public static double C(double distanceToSpeaker, double robotVelocityTowardsSpeaker) {
        return noteFallAccel * Math.pow(distanceToSpeaker, 2)
                + 2 * Math.pow(robotVelocityTowardsSpeaker, 2) * (shooterTargetZ - shooterHeight);
    }

    public static double quarticA(double robotVelocityTowardsSpeaker) {
        return 4 * Math.pow(shooterVelocity, 4)
                * (Math.pow(shooterTargetZ - shooterHeight, 2) + Math.pow(robotVelocityTowardsSpeaker, 2));
    }

    public static double quarticB(double distanceToSpeaker, double robotVelocityTowardsSpeaker) {
        return 8 * Math.pow(shooterVelocity, 3) * robotVelocityTowardsSpeaker
                * (2 * Math.pow(shooterTargetZ - shooterHeight, 2) + Math.pow(distanceToSpeaker, 2));
    }

    public static double quarticC(double distanceToSpeaker, double robotVelocityTowardsSpeaker, double C) {
        return 4 * Math.pow(shooterVelocity, 2)
                * (4 * Math.pow(shooterVelocity, 2) * Math.pow(shooterTargetZ - shooterHeight, 2)
                        + C * (shooterTargetZ - shooterHeight)
                        - Math.pow(shooterVelocity, 2) * Math.pow(distanceToSpeaker, 2)
                        + Math.pow(robotVelocityTowardsSpeaker, 2) * Math.pow(distanceToSpeaker, 2));
    }

    public static double quarticD(double distanceToSpeaker, double robotVelocityTowardsSpeaker, double C) {
        return 8 * shooterVelocity * robotVelocityTowardsSpeaker * (C * (shooterTargetZ - shooterHeight)
                - Math.pow(shooterVelocity, 2) * Math.pow(distanceToSpeaker, 2));
    }

    public static double quarticE(double distanceToSpeaker, double robotVelocityTowardsSpeaker, double C) {
        return Math.pow(C, 2) - 4 * Math.pow(shooterVelocity, 2) * Math.pow(robotVelocityTowardsSpeaker, 2)
                * Math.pow(distanceToSpeaker, 2);
    }

    public static void updateR(double quarticA, double quarticB, double quarticC, double quarticD, double quarticE) {
        R = R - ((quarticA * Math.pow(R, 4) + quarticB * Math.pow(R, 3)
                + quarticC * Math.pow(R, 2) + quarticD * R + quarticE)
                / (4 * quarticA * Math.pow(R, 3) + 3 * quarticB * Math.pow(R, 2)
                        + 2 * quarticC * R + quarticD));  
    }

    public static void resetR(){
        R = 0.0;
    }

    public static double getShooterAngle(){
        return Math.acos(R);
    }
}
