package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.subs.FollowerSubsystem;

/** FTCLib replacement for Ivy's PedroCommands.follow(follower, path). */
public class FollowPathCommand extends CommandBase {

    private final FollowerSubsystem follower;
    private final Path path;

    public FollowPathCommand(FollowerSubsystem follower, Path path) {
        this.follower = follower;
        this.path = path;
        addRequirements(follower);
    }

    @Override
    public void initialize() {
        follower.follower().follow(path);
    }

    @Override
    public boolean isFinished() {
        return !follower.follower().isBusy();
    }
}
