package org.firstinspires.ftc.teamcode.Commands.AutoCommands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Commands.CheckLoadCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeMotorCommand;
import org.firstinspires.ftc.teamcode.Commands.SavePose;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;


/**
 * This command drives the robot along a specified path while running the intake.
 * It is a sequential command group that combines path following with intake control.
 */
public class IntakeDrive extends SequentialCommandGroup {
    /**
     * Constructs an IntakeDrive command that follows a path with maxSpeed, runs the intake, waits, and then stops the intake.
     *
     * @param follower The robot's path follower.
     * @param maxSpeed The maximum speed to follow the path.
     * @param path The path chain to follow.
     * @param waitTime The time to wait in milliseconds after reaching the destination before stopping the intake.
     */
    public IntakeDrive(Follower follower, double maxSpeed, PathChain path, ColorSensor colorSensor, long waitTime) {
        addCommands(
                new FollowPathCommand(follower, path, maxSpeed).alongWith(
                        new IntakeMotorCommand(1)
                ),
                new SavePose(follower.getPose()),
                new WaitCommand(waitTime).raceWith(new CheckLoadCommand(colorSensor)),
                new IntakeMotorCommand(-1),
                new WaitCommand(20),
                new IntakeMotorCommand(0)
        );
    }

    /**
     * Constructs an IntakeDrive command that follows a path, runs the intake, waits, and then stops the intake.
     *
     * @param follower The robot's path follower.
     * @param path The path chain to follow.
     * @param waitTime The time to wait in milliseconds after reaching the destination before stopping the intake.
     */
    public IntakeDrive(Follower follower, PathChain path, long waitTime) {
        addCommands(
                new FollowPathCommand(follower, path).alongWith(
                        new IntakeMotorCommand(1)
                ),
                new SavePose(follower.getPose()),
                new WaitCommand(waitTime),
                new IntakeMotorCommand(-1),
                new WaitCommand(20),
                new IntakeMotorCommand(0)
        );
    }/**
     * Constructs an IntakeDrive command that follows a path, runs the intake, waits, and then stops the intake.
     *
     * @param follower The robot's path follower.
     * @param path The path chain to follow.
     * @param waitTime The time to wait in milliseconds after reaching the destination before stopping the intake.
     */
    /*public IntakeDrive(Follower follower, PathChain path, long waitTime) {
        addCommands(
                new FollowPathCommand(follower, path).alongWith(
                        new IntakeMotorCommand(1)
                ),
                new SavePose(follower.getPose()),
                new WaitCommand(waitTime),
                new IntakeMotorCommand(-1),
                new WaitCommand(20),
                new IntakeMotorCommand(0)
        );
    }*//**
     * Constructs an IntakeDrive command that follows a path, runs the intake, waits, and then stops the intake.
     *
     * @param follower The robot's path follower.
     * @param path The path chain to follow.
     * @param maxSpeed
     * @param intake The intake subsystem.
     * @param waitTime The time to wait in milliseconds after reaching the destination before stopping the intake.
     *//*
    public IntakeDrive(Follower follower, double maxSpeed, PathChain path, Intake intake, long waitTime) {
        addCommands(
                new FollowPathCommand(follower, path, maxSpeed).alongWith(
                        new IntakeStateCommand(intake, Intake.IntakeState.INTAKE)
                ),
                new SavePoseCommand(follower),
                new WaitCommand(waitTime),
                new IntakeStateCommand(intake, Intake.IntakeState.REVERSE),
                new WaitCommand(20),
                new IntakeStateCommand(intake, Intake.IntakeState.IDLE)
        );
    }


    *//**
     * Constructs an IntakeDrive command that follows a path and runs the intake without stopping it upon completion.
     * Use this constructor when you want the intake to continue running after the path is finished.
     *
     * @param follower The robot's path follower.
     * @param path The path chain to follow.
     * @param intake The intake subsystem.
     *//*
    public IntakeDrive(Follower follower, PathChain path, Intake intake) {
        addCommands(
                new FollowPathCommand(follower, path).alongWith(
                        new IntakeStateCommand(intake, Intake.IntakeState.INTAKE)
                ),
                new SavePoseCommand(follower)
        );
    }

    *//**
     * Constructs an IntakeDrive command that follows a path with maxSpeed and runs the intake without stopping it upon completion.
     * Use this constructor when you want the intake to continue running after the path is finished.
     *
     * @param follower The robot's path follower.
     * @param path The path chain to follow.
     * @param intake The intake subsystem.
     *//*
    public IntakeDrive(Follower follower, double maxSpeed, PathChain path, Intake intake) {
        addCommands(
                new FollowPathCommand(follower, path, maxSpeed).alongWith(
                        new IntakeStateCommand(intake, Intake.IntakeState.INTAKE)
                ),
                new SavePoseCommand(follower)
        );
    }*/
}
