package jaci.pathfinder;

import jaci.pathfinder.modifiers.SwerveModifier;

public class PathfinderJNI {
    static {
        try {
            System.loadLibrary("pathfinderjava");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static Trajectory generateTrajectory(Waypoint[] waypoints, Trajectory.Config c) {
        return new Trajectory(generateTrajectory(waypoints, c.fit, c.sample_count, c.dt, c.max_velocity, c.max_acceleration, c.max_jerk));
    }
    public static native Trajectory.Segment[] generateTrajectory(Waypoint[] waypoints, Trajectory.FitMethod fit, int samples, double dt, double maxVelocity, double maxAcceleration, double maxJerk);

    public static Trajectory[] modifyTrajectoryTank(Trajectory traj, double wheelbaseWidth) {
        Trajectory.Segment[][] mod = modifyTrajectoryTank(traj.segments, wheelbaseWidth);
        return new Trajectory[] { new Trajectory(mod[0]), new Trajectory(mod[1]) };
    }
    public static native Trajectory.Segment[][] modifyTrajectoryTank(Trajectory.Segment[] source, double wheelbaseWidth);

    public static Trajectory[] modifyTrajectorySwerve(Trajectory traj, double wheelbaseWidth, double wheelbaseDepth, SwerveModifier.Mode mode) {
        Trajectory.Segment[][] mod = modifyTrajectorySwerve(traj.segments, wheelbaseWidth, wheelbaseDepth, mode);
        return new Trajectory[] { new Trajectory(mod[0]), new Trajectory(mod[1]), new Trajectory(mod[2]), new Trajectory(mod[3]) };
    }
    public static native Trajectory.Segment[][] modifyTrajectorySwerve(Trajectory.Segment[] source, double wheelbaseWidth, double wheelbaseDepth, SwerveModifier.Mode mode);

    public static native void trajectorySerialize(Trajectory.Segment[] source, String filename);
    public static native Trajectory.Segment[] trajectoryDeserialize(String filename);

    public static native void trajectorySerializeCSV(Trajectory.Segment[] source, String filename);
    public static native Trajectory.Segment[] trajectoryDeserializeCSV(String filename);
}
