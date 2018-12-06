#include "pathfinder.h"
#include "frc/Filesystem.h"

#include <wpi/FileSystem.h>
#include <wpi/Path.h>

#include <cstdio>

class PathfinderFRC {
 public:
  PathfinderFRC() = delete;

  static constexpr double DEFAULT_ACC = 2.0;
  static constexpr double DEFAULT_JERK = 60.0;

  static std::string get_trajectory_file(std::string name) {
    wpi::SmallString<256> path;
    frc::filesystem::GetDeployDirectory(path);
    wpi::sys::path::append(path, "paths", name + ".pf1.csv");
    return std::string(path.c_str());
  }

  static int get_trajectory(std::string name, Segment *traj_out) {
    FILE *fp = fopen(get_trajectory_file(name).c_str(), "r");
    int len = pathfinder_deserialize_csv(fp, traj_out);
    fclose(fp);
    return len;
  }
};