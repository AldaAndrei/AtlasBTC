package org.firstinspires.ftc.teamcode.Commands.AutoCommands;


import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Commands.IntakeMotorCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeServoCommand;
import org.firstinspires.ftc.teamcode.Commands.SavePose;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class OpenGate extends SequentialCommandGroup {
    public OpenGate(Follower follower, PathChain path) {
        addCommands(
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path),
                        new IntakeServoCommand(RobotHardware.IntakeServoDown),
                        new IntakeMotorCommand(0)
                ),
                new WaitCommand(200),
                new IntakeServoCommand(RobotHardware.IntakeServoUp)
        );
    }
}
