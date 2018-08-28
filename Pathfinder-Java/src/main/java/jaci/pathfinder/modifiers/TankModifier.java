package jaci.pathfinder.modifiers;

import java.util.Objects;

import jaci.pathfinder.PathfinderJNI;
import jaci.pathfinder.Trajectory;

/**
 * The Tank Modifier will take in a Source Trajectory and a Wheelbase Width and spit out a
 * Trajectory for each side of the wheelbase. This is commonly used in robotics for robots which
 * have a drive system similar to a 'tank', where individual parallel sides are driven
 * independently
 * <p>
 * The Source Trajectory is measured from the centre of the drive base. The modification will not
 * modify the central trajectory
 *
 * @author Jaci
 */
public class TankModifier {
    Trajectory source, left, right;

    /**
     * Create an instance of the modifier
     *
     * @param source The source (center) trajectory
     */
    public TankModifier(Trajectory source) {
        this.source = source;
    }

    /**
     * Generate the Trajectory Modification
     *
     * @param wheelbaseWidth The width (in meters) between the individual sides of the drivebase
     * @return self
     */
    public TankModifier modify(double wheelbaseWidth) {
        Trajectory[] trajs = PathfinderJNI.modifyTrajectoryTank(source, wheelbaseWidth);
        left = trajs[0];
        right = trajs[1];
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
     * Get the trajectory for the left side of the drive base
     * @return a trajectory for the left side
     */
    public Trajectory getLeftTrajectory() {
        return left;
    }

    /**
     * Get the trajectory for the right side of the drive base
     * @return a trajectory for the right side
     */
    public Trajectory getRightTrajectory() {
        return right;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        TankModifier modifier = (TankModifier) o;

        return Objects.equals(source, modifier.source) &&
                Objects.equals(left, modifier.left) &&
                Objects.equals(right, modifier.right);
    }

    @Override
    public int hashCode() {
        return Objects.hash(source, left, right);
    }

    @Override
    public String toString() {
        return "TankModifier{" +
                "source=" + source +
                ", left=" + left +
                ", right=" + right +
                '}';
    }
}
