package jaci.pathfinder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * PathDriveTrain is an abstraction used to pass odometry in and motor values out
 */
public interface PathDriveTrain {
    public int getLeftEncoderTicks();
    public int getRightEncoderTicks();
    public void setMotors(double left, double right);
    public default double getHeading() {
        return Double.NaN;
    }
    static PathDriveTrain fromParts(Encoder leftEncoder, Encoder rightEncoder, DifferentialDrive drive) {
        return new PathDriveTrain() {
            @Override
            public int getLeftEncoderTicks() {
                return leftEncoder.get();
            }
            @Override
            public int getRightEncoderTicks() {
                return rightEncoder.get();
            }
            @Override
            public void setMotors(double left, double right) {
                drive.tankDrive(left,right);
            }
        };
    }
}