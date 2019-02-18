#pragma once

#include "pathfinder.h"
#include "frc/Filesystem.h"

#include <wpi/FileSystem.h>
#include <wpi/Path.h>

#include <cstdio>
#include <stdexcept>

/**
 * Utility class for using Pathfinder v1 with WPILib for FRC.
 */
class PathfinderFRC {
 public:
  PathfinderFRC() = delete;

  /**
   * Default Acceleration for a Kit-of-parts drivetrain (in m/s/s)
   */
  static constexpr double DEFAULT_ACC = 2.0;

  /**
   * Default Jerk, in m/s/s/s
   */
  static constexpr double DEFAULT_JERK = 60.0;

  /**
   * Get the absolute path of a trajectory file generated with the given name, usually by PathWeaver.
   * This looks in the deploy directory, e.g. for name "testtraj", "/home/lvuser/deploy/paths/testtraj.pf1.csv"
   * is the result (placed in "src/main/deploy/paths/testtraj.pf1.csv" in your project directory).
   * 
   * @param name The name of the path
   * @return The absolute path of the trajectory
   */
  static std::string get_trajectory_file(std::string name) {
    wpi::SmallString<256> path;
    frc::filesystem::GetDeployDirectory(path);
    wpi::sys::path::append(path, "paths", name + ".pf1.csv");
    if (!wpi::sys::fs::exists(path)) {
      throw std::runtime_error("Path " + std::string(path.c_str()) + " does not exist!");
    }
    return std::string(path.c_str());
  }

  /**
   * Load a Trajectory from file, at the path described by \ref get_trajectory_file(std::string)
   * 
   * @param name The name of the path
   * @param traj_out Pre-allocated Trajectory Segment buffer
   * @return The length of the path (number of Segments)
   */
  static int get_trajectory(std::string name, Segment *traj_out) {
    FILE *fp = fopen(get_trajectory_file(name).c_str(), "r");
    int len = pathfinder_deserialize_csv(fp, traj_out);
    fclose(fp);
    return len;
  }
};