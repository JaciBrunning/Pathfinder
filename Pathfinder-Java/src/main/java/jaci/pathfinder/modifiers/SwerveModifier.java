package jaci.pathfinder.modifiers;

import jaci.pathfinder.PathfinderJNI;
import jaci.pathfinder.Trajectory;

/**
 * The Swerve Modifier will take in a Source Trajectory and spit out 4 trajectories, 1 for each wheel on the drive.
 * This is commonly used in robotics for robots with 4 individual wheels in a 'swerve' configuration, where each wheel
 * can rotate to a specified heading while still being powered.
 *
 * The Source Trajectory is measured from the centre of the drive base. The modification will not modify the central
 * trajectory
 *
 * @author Jaci
 */
public class SwerveModifier {

    /**
     * The Swerve Mode to generate the trajectory with. This declares how the heading
     * of the robot is calculated based on the central trajectory.
     */
    public static enum Mode {
        /**
         * SWERVE_DEFAULT will generate a trajectory with the robot constantly
         * facing forward, translating from side to side in order to follow
         * a curved path
         */
        SWERVE_DEFAULT
    }

    Trajectory source, fl, fr, bl, br;

    /**
     * Create an instance of the modifier
     * @param source The source (center) trajectory
     */
    public SwerveModifier(Trajectory source) {
        this.source = source;
    }

    /**
     * Generate the Trajectory Modification
     * @param wheelbase_width   The width (in meters) between the individual left-right sides of the drivebase
     * @param wheelbase_depth   The width (in meters) between the individual front-back sides of the drivebase
     * @param mode              The SwerveMode to use for generation
     * @return                  self
     */
    public SwerveModifier modify(double wheelbase_width, double wheelbase_depth, Mode mode) {
        Trajectory[] trajs = PathfinderJNI.modifyTrajectorySwerve(source, wheelbase_width, wheelbase_depth, mode);
        fl = trajs[0];
        fr = trajs[1];
        bl = trajs[2];
        br = trajs[3];
        return this;
    }

    /**
     * Get the initial source trajectory
     * @return the source trajectory
     */
    public Trajectory getSourceTrajectory() {
        return source;
    }

    /**
     * Get the trajectory for the front-left wheel of the drive base
     * @return a trajectory for the front left
     */
    public Trajectory getFrontLeftTrajectory() {
        return fl;
    }

    /**
     * Get the trajectory for the front-right wheel of the drive base
     * @return a trajectory for the front right
     */
    public Trajectory getFrontRightTrajectory() {
        return fr;
    }

    /**
     * Get the trajectory for the back-left wheel of the drive base
     * @return a trajectory for the back left
     */
    public Trajectory getBackLeftTrajectory() {
        return bl;
    }

    /**
     * Get the trajectory for the back-right wheel of the drive base
     * @return a trajectory for the back right
     */
    public Trajectory getBackRightTrajectory() {
        return br;
    }

}
