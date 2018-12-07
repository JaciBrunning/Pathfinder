package jaci.pathfinder;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Command class for using a PathFollower in the context of a Command based robot.
 */

public class PathFollowerCommand extends Command {

    private PathFollower pf;
    public PathFollowerCommand(String pathName, DriveSubsystem drive, PathFollowerConfig cfg) {
        requires(drive);
        pf = new PathFollower(pathName,drive,cfg);
    }

    @Override
    public void execute() {
        pf.run();
    }

    public boolean isFinished() {
        return pf.isFinished();
    }
}