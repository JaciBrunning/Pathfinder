package jaci.pathfinder;

/**
 * A single waypoint used for Trajectory Generation.
 * <p>
 * A Waypoint is a 'setpoint' that you wish for your trajectory to intersect. Waypoints are given an
 * X, Y coordinate stating their location in space, and an exit angle that defines the heading the
 * trajectory should point towards once this waypoint is reached. The angle is given in Radians
 *
 * @author Jaci
 */
public class Waypoint {
    public double x, y, angle;

    /**
     * Create a Waypoint (setpoint) for Trajectory Generation
     *
     * @param x     The X position of the waypoint in meters
     * @param y     The Y position of the waypoint in meters
     * @param angle The exit angle of the waypoint in radians
     */
    public Waypoint(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

}
