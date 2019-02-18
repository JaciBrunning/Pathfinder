package jaci.pathfinder;

import java.io.File;
import java.io.IOException;

/**
 * The main class of the Pathfinder Library. The Pathfinder Library is used for Motion Profile and Trajectory Generation.
 *
 * This class contains some of the most common methods you will use when doing Motion Profiling
 *
 * @author Jaci
 */
public class Pathfinder {

    /**
     * Convert degrees to radians. This is included here for static imports. In this library, all angle values are
     * given in radians
     * @param degrees the degrees input
     * @return the input in radians
     */
    public static double d2r(double degrees) {
        return Math.toRadians(degrees);
    }

    /**
     * Convert radians to degrees. This is included here for static imports.
     * @param radians the radians input
     * @return the input in degrees
     */
    public static double r2d(double radians) {
        return Math.toDegrees(radians);
    }

    /**
     * Bound an angle (in degrees) to -180 to 180 degrees.
	 * @param angle_degrees an input angle in degrees
	 * @return the bounded angle
     */
    public static double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        return angle_degrees;
    }

    /**
     * Generate a motion profile trajectory using the given waypoints and configuration.
     * @param waypoints     An array of waypoints (setpoints) for the trajectory path to intersect
     * @param config        The configuration of the trajectory, including max velocity, acceleration, jerk
     *                      and other values such as time scale and fit method
     * @return              The generated trajectory (an array of segments)
     */
    public static Trajectory generate(Waypoint[] waypoints, Trajectory.Config config) {
        return PathfinderJNI.generateTrajectory(waypoints, config);
    }

    /**
     * Write the Trajectory to a Binary (non human readable) file
     * @param file          The file to write to
     * @param trajectory    The trajectory to write
     */
    public static void writeToFile(File file, Trajectory trajectory) {
        file.getParentFile().mkdirs();
        PathfinderJNI.trajectorySerialize(trajectory.segments, file.getAbsolutePath());
    }

    /**
     * Read a Trajectory from a Binary (non human readable) file
     * @param file          The file to read from
     * @return              The trajectory that was read from file
     */
    public static Trajectory readFromFile(File file) throws IOException {
        if (!file.exists())
            throw new IOException("File " + file.getAbsolutePath() + " does not exist!");
        return new Trajectory(PathfinderJNI.trajectoryDeserialize(file.getAbsolutePath()));
    }

    /**
     * Write the Trajectory to a CSV File
     * @param file          The file to write to
     * @param trajectory    The trajectory to write
     */
    public static void writeToCSV(File file, Trajectory trajectory) {
        file.getParentFile().mkdirs();
        PathfinderJNI.trajectorySerializeCSV(trajectory.segments, file.getAbsolutePath());
    }

    /**
     * Read a Trajectory from a CSV File
     * 
     * @param file The file to read from
     * @return The trajectory that was read from file
     */
    public static Trajectory readFromCSV(File file) throws IOException {
        if (!file.exists())
            throw new IOException("File " + file.getAbsolutePath() + " does not exist!");
        return new Trajectory(PathfinderJNI.trajectoryDeserializeCSV(file.getAbsolutePath()));
    }

    /**
     * Thrown when a Trajectory could not be generated for an unknown reason.
     */
    public static class GenerationException extends Exception {
        public GenerationException(String message) {
            super(message);
        }
    }

}
