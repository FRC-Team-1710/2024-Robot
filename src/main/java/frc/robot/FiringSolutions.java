package frc.robot;

public final class FiringSolutions {
    private static final double shooterHeight = 0.325;
    private static final double noteFallAccel = 9.8;
    private static final double shooterTargetXBlue = 0.0;
    private static final double shooterTargetXRed = 16.54;
    private static final double shooterTargetY = 8.02;
    private static final double shooterTargetZ = 2.05;

    private static double shooterTargetX;

    private FiringSolutions(){
    }

    public static void setAlliance(boolean redAlliance){
        if (redAlliance){
            shooterTargetX = shooterTargetXRed;
        } else {
            shooterTargetX = shooterTargetXBlue;
        }
    }

    public static double getAngleToSpeaker(double robotX, double robotY){
        return Math.atan((shooterTargetY - robotY) / (shooterTargetX - robotX));
    }

    public static double getDistanceToSpeaker(double robotX, double robotY){
        return Math.sqrt(Math.pow(shooterTargetX - robotX, 2) + Math.pow(shooterTargetY - robotY, 2));
    }

    public static double getRobotVelocityTowardsSpeaker(double robotVelocityX, double robotVelocityY, double angleToSpeaker, double robotHeading){
        return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2)) * Math.cos(Math.atan(robotVelocityY / robotVelocityX) - angleToSpeaker - robotHeading);
    }

    public static double getRobotVelocityPerpendicularToSpeaker(double robotVelocityX, double robotVelocityY, double angleToSpeaker, double robotHeading){
        return Math.sqrt(Math.pow(robotVelocityX, 2) + Math.pow(robotVelocityY, 2)) * Math.sin(Math.atan(robotVelocityY / robotVelocityX) - angleToSpeaker - robotHeading);
    }

    public static double getShooterVelocityX(double robotX, double robotY){
        return getDistanceToSpeaker(robotX, robotY) / Math.sqrt((2 * (shooterTargetZ - shooterHeight)) / noteFallAccel);
    }

    public static double getShooterVelocityZ(){
        return Math.sqrt(2 * noteFallAccel * (shooterTargetZ - shooterHeight));
    }

    public static double getShooterVelocity(double shooterVelocityX, double shooterVelocityZ, double robotVelocityTowardsSpeaker, double robotVelocityPerpendicularToSpeaker){
        return Math.sqrt(Math.pow(shooterVelocityX - robotVelocityTowardsSpeaker, 2) + Math.pow(shooterVelocityZ, 2) + Math.pow(robotVelocityPerpendicularToSpeaker, 2));
    }

    public static double getShooterAngle(double shooterVelocityX, double shooterVelocityZ, double robotVelocityTowardsSpeaker){
        return Math.atan(shooterVelocityZ / shooterVelocityX - robotVelocityTowardsSpeaker);
    }
    
    public static double getRobotOffsetAngle(double robotVelocityPerpendicularToSpeaker, double shooterVelocityX){
        return Math.atan(robotVelocityPerpendicularToSpeaker / shooterVelocityX);
    }

}
