package jaci.pathfinder;

import java.util.Arrays;
import java.util.Objects;

/**
 * The Trajectory object contains an array of Segments that represent the location, velocity,
 * acceleration, jerk and heading of a particular point in the trajectory.
 * <p>
 * Trajectories can be generated with the Pathfinder class
 *
 * @author Jaci
 */
public class Trajectory {
    public Segment[] segments;

    public Trajectory(Segment[] segments) {
        this.segments = segments;
    }

    public Trajectory(Trajectory other) {
        segments = new Segment[other.length()];
        for (int i = 0; i < other.length(); i++) {
            segments[i] = new Segment(other.get(i));
        }
    }

    /** @deprecated use {@link #Trajectory(Trajectory)} instead */
    @Deprecated
    public Trajectory(int length) {
        segments = new Segment[length];
    }

    public Segment get(int index) {
        return segments[index];
    }

    public int length() {
        return segments.length;
    }

    /** @deprecated use {@link #Trajectory(Trajectory)} instead */
    @Deprecated
    public Trajectory copy() {
        Trajectory toCopy = new Trajectory(length());
        for (int i = 0; i < length(); i++) {
            toCopy.segments[i] = new Segment(get(i));
        }
        return toCopy;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Trajectory that = (Trajectory) o;

        return Arrays.equals(segments, that.segments);
    }

    @Override
    public int hashCode() {
        return Arrays.hashCode(segments);
    }

    @Override
    public String toString() {
        return "Trajectory{" +
                "segments=" + Arrays.toString(segments) +
                '}';
    }

    /**
     * The Fit Method defines the function by which the trajectory path is generated. By default,
     * the HERMITE_CUBIC method is used.
     */
    public enum FitMethod {
        HERMITE_CUBIC, HERMITE_QUINTIC;
    }

    /**
     * The Trajectory Configuration outlines the rules to follow while generating the trajectory.
     * This includes the method used for 'fitting' the spline, the amount of samples to use, the
     * time difference and maximum values for the velocity, acceleration and jerk of the
     * trajectory.
     */
    public static class Config {

        public static final int SAMPLES_FAST = 1000;
        public static final int SAMPLES_LOW = SAMPLES_FAST * 10;
        public static final int SAMPLES_HIGH = SAMPLES_LOW * 10;

        public FitMethod fit;
        public int sample_count;
        public double dt, max_velocity, max_acceleration, max_jerk;

        /**
         * Create a Trajectory Configuration
         *
         * @param fit             The fit method to use
         * @param samples         How many samples to use to refine the path (higher = smoother,
         *                        lower = faster)
         * @param dt              The time delta between points (in seconds)
         * @param maxVelocity     The maximum velocity the body is capable of travelling at (in
         *                        meters per second)
         * @param maxAcceleration The maximum acceleration to use (in meters per second per second)
         * @param maxJerk         The maximum jerk (acceleration per second) to use
         */
        public Config(FitMethod fit,
                      int samples,
                      double dt,
                      double maxVelocity,
                      double maxAcceleration,
                      double maxJerk) {
            this.fit = fit;
            this.sample_count = samples;
            this.dt = dt;
            this.max_velocity = maxVelocity;
            this.max_acceleration = maxAcceleration;
            this.max_jerk = maxJerk;
        }

        @Override
        public String toString() {
            return "Config{" +
                    "fit=" + fit +
                    ", sample_count=" + sample_count +
                    ", dt=" + dt +
                    ", max_velocity=" + max_velocity +
                    ", max_acceleration=" + max_acceleration +
                    ", max_jerk=" + max_jerk +
                    '}';
        }
    }

    /**
     * A Trajectory Segment is a particular point in a trajectory. The segment contains the xy
     * position and the velocity, acceleration, jerk and heading at this point
     */
    public static class Segment {
        public double dt, x, y, position, velocity, acceleration, jerk, heading;

        public Segment(double dt,
                       double x,
                       double y,
                       double position,
                       double velocity,
                       double acceleration,
                       double jerk,
                       double heading) {
            this.dt = dt;
            this.x = x;
            this.y = y;
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.jerk = jerk;
            this.heading = heading;
        }

        public Segment(Segment other) {
            dt = other.dt;
            x = other.x;
            y = other.y;
            position = other.position;
            velocity = other.velocity;
            acceleration = other.acceleration;
            jerk = other.jerk;
            heading = other.heading;
        }

        /** @deprecated use {@link #Segment(Segment)} instead */
        @Deprecated
        public Segment copy() {
            return new Segment(this);
        }

        public boolean fuzzyEquals(Segment seg) {
            return ae(seg.dt, dt)
                    && ae(seg.x, x) && ae(seg.y, y)
                    && ae(seg.position, position) && ae(seg.velocity, velocity)
                    && ae(seg.acceleration, acceleration) && ae(seg.jerk, jerk)
                    && ae(seg.heading, heading);
        }

        private boolean ae(double one, double two) {
            return Math.abs(one - two) < 0.0001; // Really small value
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;

            Segment segment = (Segment) o;

            return Double.compare(segment.dt, dt) == 0 &&
                    Double.compare(segment.x, x) == 0 &&
                    Double.compare(segment.y, y) == 0 &&
                    Double.compare(segment.position, position) == 0 &&
                    Double.compare(segment.velocity, velocity) == 0 &&
                    Double.compare(segment.acceleration, acceleration) == 0 &&
                    Double.compare(segment.jerk, jerk) == 0 &&
                    Double.compare(segment.heading, heading) == 0;
        }

        @Override
        public int hashCode() {
            return Objects.hash(dt, x, y, position, velocity, acceleration, jerk, heading);
        }

        @Override
        public String toString() {
            return "Segment{" +
                    "dt=" + dt +
                    ", x=" + x +
                    ", y=" + y +
                    ", position=" + position +
                    ", velocity=" + velocity +
                    ", acceleration=" + acceleration +
                    ", jerk=" + jerk +
                    ", heading=" + heading +
                    '}';
        }
    }
}
