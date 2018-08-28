package jaci.pathfinder.followers;

import java.util.Objects;

import jaci.pathfinder.Trajectory;

/**
 * The EncoderFollower is an object designed to follow a trajectory based on encoder input. This
 * class can be used for Tank or Swerve drive implementations.
 *
 * @author Jaci
 */
public class EncoderFollower {
    int encoder_offset, encoder_tick_count;
    double wheel_circumference;

    double kp, ki, kd, kv, ka;

    double last_error, heading;

    int segment;
    Trajectory trajectory;

    public EncoderFollower(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    public EncoderFollower() { }

    /**
     * Set a new trajectory to follow, and reset the cumulative errors and segment counts
     * @param traj a previously generated trajectory
     */
    public void setTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
        reset();
    }

    /**
     * Configure the PID/VA Variables for the Follower
     *
     * @param kp The proportional term. This is usually quite high (0.8 - 1.0 are common values)
     * @param ki The integral term. Currently unused.
     * @param kd The derivative term. Adjust this if you are unhappy with the tracking of the
     *           follower. 0.0 is the default
     * @param kv The velocity ratio. This should be 1 over your maximum velocity @ 100% throttle.
     *           This converts m/s given by the algorithm to a scale of -1..1 to be used by your
     *           motor controllers
     * @param ka The acceleration term. Adjust this if you want to reach higher or lower speeds
     *           faster. 0.0 is the default
     */
    public void configurePIDVA(double kp, double ki, double kd, double kv, double ka) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kv = kv;
        this.ka = ka;
    }

    /**
     * Configure the Encoders being used in the follower.
     *
     * @param initialPosition    The initial 'offset' of your encoder. This should be set to the
     *                           encoder value just before you start to track
     * @param ticksPerRevolution How many ticks per revolution the encoder has
     * @param wheelDiameter      The diameter of your wheels (or pulleys for track systems) in
     *                           meters
     */
    public void configureEncoder(int initialPosition,
                                 int ticksPerRevolution,
                                 double wheelDiameter) {
        encoder_offset = initialPosition;
        encoder_tick_count = ticksPerRevolution;
        wheel_circumference = Math.PI * wheelDiameter;
    }

    /**
     * Reset the follower to start again. Encoders must be reconfigured.
     */
    public void reset() {
        last_error = 0;
        segment = 0;
    }

    /**
     * Calculate the desired output for the motors, based on the amount of ticks the encoder has
     * gone through. This does not account for heading of the robot. To account for heading, add
     * some extra terms in your control loop for realignment based on gyroscope input and the
     * desired heading given by this object.
     *
     * @param encoderTick The amount of ticks the encoder has currently measured.
     * @return The desired output for your motor controller
     */
    public double calculate(int encoderTick) {
        // Number of Revolutions * Wheel Circumference
        double distance_covered = ((double) (encoderTick - encoder_offset) / encoder_tick_count)
                * wheel_circumference;
        if (segment < trajectory.length()) {
            Trajectory.Segment seg = trajectory.get(segment);
            double error = seg.position - distance_covered;
            double calculated_value =
                    kp * error +                                    // Proportional
                            kd * ((error - last_error) / seg.dt) +          // Derivative
                            (kv * seg.velocity + ka * seg.acceleration);    // V and A Terms
            last_error = error;
            heading = seg.heading;
            segment++;

            return calculated_value;
        } else {
            return 0;
        }
    }

    /**
     * @return the desired heading of the current point in the trajectory
     */
    public double getHeading() {
        return heading;
    }

    /**
     * @return the current segment being operated on
     */
    public Trajectory.Segment getSegment() {
        return trajectory.get(segment);
    }

    /**
     * @return whether we have finished tracking this trajectory or not.
     */
    public boolean isFinished() {
        return segment >= trajectory.length();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        EncoderFollower follower = (EncoderFollower) o;

        return encoder_offset == follower.encoder_offset &&
                encoder_tick_count == follower.encoder_tick_count &&
                Double.compare(follower.wheel_circumference, wheel_circumference) == 0 &&
                Double.compare(follower.kp, kp) == 0 &&
                Double.compare(follower.ki, ki) == 0 &&
                Double.compare(follower.kd, kd) == 0 &&
                Double.compare(follower.kv, kv) == 0 &&
                Double.compare(follower.ka, ka) == 0 &&
                Double.compare(follower.last_error, last_error) == 0 &&
                Double.compare(follower.heading, heading) == 0 &&
                segment == follower.segment &&
                Objects.equals(trajectory, follower.trajectory);
    }

    @Override
    public int hashCode() {
        return Objects.hash(encoder_offset,
                            encoder_tick_count,
                            wheel_circumference,
                            kp,
                            ki,
                            kd,
                            kv,
                            ka,
                            last_error,
                            heading,
                            segment,
                            trajectory);
    }

    @Override
    public String toString() {
        return "EncoderFollower{" +
                "encoder_offset=" + encoder_offset +
                ", encoder_tick_count=" + encoder_tick_count +
                ", wheel_circumference=" + wheel_circumference +
                ", kp=" + kp +
                ", ki=" + ki +
                ", kd=" + kd +
                ", kv=" + kv +
                ", ka=" + ka +
                ", last_error=" + last_error +
                ", heading=" + heading +
                ", segment=" + segment +
                ", trajectory=" + trajectory +
                '}';
    }
}
