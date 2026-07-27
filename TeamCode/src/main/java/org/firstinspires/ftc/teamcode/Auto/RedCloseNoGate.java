package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Commands.AutoCommands.*;
import org.firstinspires.ftc.teamcode.Commands.SavePose;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(group = "redclose", name = "red close no gate test")
public class RedCloseNoGate extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    // Define Poses cleanly at the top level
    private final Pose startPose = new Pose(117, 126.534, Math.toRadians(0));
    private final Pose shooting_pose = new Pose(88.000, 84.000, Math.toRadians(0));
    private final Pose gate = new Pose(127, 72.66, Math.toRadians(0));
    private final Pose spike1 = new Pose(123.000, 84.000, Math.toRadians(0));
    private final Pose spike2 = new Pose(121.000, 58.000, Math.toRadians(0));
    private final Pose spike3 = new Pose(121.000, 35.000, Math.toRadians(0));
    private final Pose exit_zone = new Pose(112.854, 82.732, Math.toRadians(0));

    // Control Points for Curves
    private final Pose ctrl_spike1 = new Pose(71.000, 55.000);
    private final Pose ctrl_spike2 = new Pose(71.000, 55.000);
    private final Pose ctrl_spike3 = new Pose(72.000, 32.000);

    private PathChain shoot_preload, pickup_spike1, open_gate1, shoot_spike1, pickup_spike2, shoot_spike2, pickup_spike3, shoot_spike3, leave_zone;

    public void buildPaths() {
        // Path 1
        shoot_preload = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shooting_pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 2
        pickup_spike1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shooting_pose, spike1))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        open_gate1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(spike1, gate))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 3
        shoot_spike1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(gate, shooting_pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 4
        pickup_spike2 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shooting_pose, ctrl_spike2, spike2))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 5
        shoot_spike2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(spike2, shooting_pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 6
        pickup_spike3 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shooting_pose, ctrl_spike3, spike3))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 7
        shoot_spike3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(spike3, shooting_pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 8
        leave_zone = robot.follower.pathBuilder()
                .addPath(new BezierLine(shooting_pose, exit_zone))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    @Override
    public void initialize() {
        super.reset();
        robot.init(hardwareMap, true);
        robot.updateAlliance(RobotHardware.AllianceColor.RED);



        // Initialize Follower via your updated Constants
        robot.follower = Constants.createFollower(hardwareMap);
        robot.follower.setStartingPose(startPose);

        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new DriveShoot(robot.follower, shoot_preload,500),
                new IntakeDrive(robot.follower, pickup_spike1, 500),
                new OpenGate(robot.follower, open_gate1),
                new DriveShoot(robot.follower, shoot_spike1, 500),
                new IntakeDrive(robot.follower, pickup_spike2, 500),
                new DriveShoot(robot.follower, shoot_spike2, 500),
                new IntakeDrive(robot.follower, pickup_spike3, 500),
                new DriveShoot(robot.follower, shoot_spike3, 500),
                new FollowPathCommand(robot.follower, leave_zone),
                new SavePose(robot.follower.getPose())
        );

        schedule(autonomousSequence);
    }

    @Override
    public void initialize_loop(){
        telemetry.addLine("OpMode selected");
    }

    @Override
    public void run() {
        super.run();

        robot.follower.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("x", robot.follower.getPose().getX());
        telemetry.addData("y", robot.follower.getPose().getY());
        telemetry.addData("heading", robot.follower.getPose().getHeading());
        telemetry.addData("Busy", robot.follower.isBusy());
        telemetry.update();
    }
}
