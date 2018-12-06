import java.io.File;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Pathfinder;
import edu.wpi.first.wpilibj.Filesystem;

public class PathfinderFRC {
  private PathfinderFRC() {}

  public static double DEFAULT_ACC  = 2.0;
  public static double DEFAULT_JERK = 60.0;

  public static File getTrajectoryFile(String name) {
    return new File(Filesystem.getDeployDirectory(), "paths/" + name + ".pf1.csv");
  }

  public static Trajectory getTrajectory(String name) {
    return Pathfinder.readFromCSV(getTrajectoryFile(name));
  }

}