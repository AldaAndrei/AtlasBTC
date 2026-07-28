package org.firstinspires.ftc.teamcode.Commands.AutoCommands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Commands.SavePose;
import org.firstinspires.ftc.teamcode.Commands.Shoot;
import org.firstinspires.ftc.teamcode.Commands.StopShoot;


/**
 * A command that spools up the launcher while driving along a path, then stops and shoots.
 */
public class DriveShoot extends SequentialCommandGroup {
    /**
     * Constructs a SpoolDriveShoot command.
     *
     * @param follower The follower to use for path following.
     * @param path The path to follow.The limelight subsystem.
     * @param waitBeforeStop The time to wait in milliseconds after shooting before stopping the launcher and turret.
     */
    public DriveShoot(Follower follower, PathChain path, long waitBeforeStop){
        addCommands(
                new FollowPathCommand(follower, path),
                new SavePose(follower),
                new WaitCommand(400),
                new Shoot(),
                new WaitCommand(waitBeforeStop),
                new StopShoot()
        );
    }

    /**
     * Constructs a SpoolDriveShoot command with a specified maximum speed.
     *
     * @param follower The follower to use for path following.
     * @param maxSpeed The maximum speed to follow the path at.
     * @param path The path to follow.
    The limelight subsystem.
     * @param waitBeforeStop The time to wait in milliseconds after shooting before stopping the launcher and turret.
     */
    public DriveShoot(Follower follower, double maxSpeed, PathChain path, long waitBeforeStop){
        addCommands(
                new FollowPathCommand(follower, path, maxSpeed),
                new SavePose(follower),
                new WaitCommand(200),
                new Shoot(),
                new WaitCommand(waitBeforeStop),
                new StopShoot()
        );
    }
}
