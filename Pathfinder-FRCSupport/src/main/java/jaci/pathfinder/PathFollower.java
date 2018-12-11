package jaci.pathfinder;
/**
 * Implement path following logic using PFv1.
 */
import jaci.pathfinder.followers.EncoderFollower;

public class PathFollower {


    private Trajectory leftTrajectory;
    private Trajectory rightTrajectory;

    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;

    private PathFollowerConfig config;

    private PathDriveTrain driveBase;

    public PathFollower(String csvPath, PathDriveTrain driveBase, PathFollowerConfig config) {
        leftTrajectory = PathfinderFRC.getTrajectory(csvPath+".left");
        rightTrajectory = PathfinderFRC.getTrajectory(csvPath+".right");
        this.config = config;
        this.driveBase = driveBase;
        leftFollower = new EncoderFollower();
        leftFollower.configureEncoder(0, config.encoderCPR, config.wheelDiameter);
        leftFollower.configurePIDVA(config.kP, config.kI, config.kD, config.kV, config.kA);
        leftFollower.setTrajectory(leftTrajectory);

        rightFollower = new EncoderFollower();
        rightFollower.configureEncoder(0, config.encoderCPR, config.wheelDiameter);
        rightFollower.configurePIDVA(config.kP, config.kI, config.kD, config.kV, config.kA);
        rightFollower.setTrajectory(rightTrajectory);

    }

    public void run() {
        double leftOut = leftFollower.calculate(driveBase.getLeftEncoderTicks());
        double rightOut = rightFollower.calculate(driveBase.getRightEncoderTicks());
        double gyroHeading = driveBase.getHeading();
        if(Double.isNaN(gyroHeading)) {
            driveBase.setMotors(leftOut, rightOut);
        } else {
            double angleDiff = Pathfinder.boundHalfDegrees(Pathfinder.r2d(leftFollower.getHeading()) -gyroHeading);
            driveBase.setMotors(leftOut - config.gyroP * angleDiff, rightOut + config.gyroP * angleDiff);
        }
    }

    public boolean isFinished() {
        return leftFollower.isFinished() || rightFollower.isFinished();
    }

    public void reset() {
        leftFollower.reset();
        rightFollower.reset();
        leftFollower.configureEncoder(driveBase.getLeftEncoderTicks(),config.encoderCPR,config.wheelDiameter);
        rightFollower.configureEncoder(driveBase.getRightEncoderTicks(),config.encoderCPR,config.wheelDiameter);
    }

}