package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.Commands.IntakeMotorCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeServoCommand;
import org.firstinspires.ftc.teamcode.Commands.NimicCommand;
import org.firstinspires.ftc.teamcode.Commands.RumbleOnBallCommand;
import org.firstinspires.ftc.teamcode.Commands.Shoot;
import org.firstinspires.ftc.teamcode.Commands.StopShoot;
import org.firstinspires.ftc.teamcode.Commands.StopperServoCommand;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@Config
@TeleOp(name = "TeleOp")
public class Tele0p extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();

    public static GamepadEx gamepadEx;
    public static GamepadEx gamepadEx2;
    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;
    double x;
    double y;
    double rx;
    double loopTime;
    public static int manualVelocity = 0;
    public static double manualAngle = 0;


    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, false);

        //*shoot
        Trigger rightTrigger = new Trigger(() -> gamepad1.right_trigger > 0.1);

        // Left trigger: spool the flywheel (gamepad1 driver's job)
        rightTrigger.whileActiveOnce(
                new Shoot()
        ).whenInactive(
                new StopShoot()
        );

        //*intake
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld( new SequentialCommandGroup(new IntakeMotorCommand(1),
                        new IntakeServoCommand(RobotHardware.IntakeServoDown)))
                .whenReleased( new SequentialCommandGroup(new IntakeMotorCommand(0),
                        new IntakeServoCommand(RobotHardware.IntakeServoUp)));

        //*Intake reverse
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                        .whileHeld( new SequentialCommandGroup(new IntakeMotorCommand(-1),
                                new IntakeServoCommand(RobotHardware.IntakeServoDown)))
                        .whenReleased( new SequentialCommandGroup(new IntakeMotorCommand(0),
                                new IntakeServoCommand(RobotHardware.IntakeServoUp)));

        //*alliance
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new InstantCommand(() ->  {
                    robot.updateAlliance(RobotHardware.AllianceColor.BLUE);
                }
                )
        );
        //setare manuala alianta rosie
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new InstantCommand(() ->  {
                    robot.updateAlliance(RobotHardware.AllianceColor.RED);
                }
                )
        );
        gamepadEx.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(new SequentialCommandGroup(
                                new InstantCommand(() -> robot.follower.setPose(RobotHardware.ResetFarPose)),
//                        new ConditionalCommand(
//                                new InstantCommand(() -> robot.follower.setPose(RobotHardware.ResetPose)),
//                                new InstantCommand(() -> robot.follower.setPose(RobotHardware.ResetFarPose)),
//                                () -> true), // robot.follower.getPose().getY() >= 32
                                new InstantCommand(() -> gamepad1.rumble(1000)),
                                new InstantCommand(()-> manualVelocity = 0),
                                new InstantCommand(()-> manualAngle = 0)
                        )
                );

        gamepadEx.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> robot.follower.setPose(RobotHardware.ResetPose)),
//                        new ConditionalCommand(
//                                new InstantCommand(() -> robot.follower.setPose(RobotHardware.ResetPose)),
//                                new InstantCommand(() -> robot.follower.setPose(RobotHardware.ResetFarPose)),
//                                () -> true), // robot.follower.getPose().getY() >= 32
                        new InstantCommand(() -> gamepad1.rumble(1000)),
                        new InstantCommand(()-> manualVelocity = 0),
                        new InstantCommand(()-> manualAngle = 0)
                        )
                );

        gamepadEx.getGamepadButton(GamepadKeys.Button.SHARE)
                .whenPressed(new SequentialCommandGroup(
                                new InstantCommand(() -> robot.follower.setPose(RobotHardware.ResetHumanPose)),
//                        new ConditionalCommand(
//                                new InstantCommand(() -> robot.follower.setPose(RobotHardware.ResetPose)),
//                                new InstantCommand(() -> robot.follower.setPose(RobotHardware.ResetFarPose)),
//                                () -> true), // robot.follower.getPose().getY() >= 32
                                new InstantCommand(() -> gamepad1.rumble(1000)),
                                new InstantCommand(()-> manualVelocity = 0),
                                new InstantCommand(()-> manualAngle = 0)
                        )
                );


        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(()-> manualVelocity += 10);

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(()-> manualVelocity -= 10);

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(()-> manualAngle += Math.toRadians(2));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(()-> manualAngle -= Math.toRadians(2));


        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(()-> manualVelocity += 10);

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(()-> manualVelocity -= 10);

        gamepadEx2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(()-> manualAngle += Math.toRadians(2));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(()-> manualAngle -= Math.toRadians(2));

        RumbleOnBallCommand rumbleCommand = new RumbleOnBallCommand(robot.ColorSensor, gamepad1, gamepad2);



        while(opModeInInit() && !isStopRequested()){
        }

        robot.IntakeServo.setPosition(RobotHardware.IntakeServoUp);
        robot.StopperServo.setPosition(RobotHardware.StopperServoClosed);
        robot.StopperServo2.setPosition(RobotHardware.StopperServoClosed);
        robot.TurretAngleServo.setPosition(0.5);


        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x * 1.1;
        rx = gamepad1.right_stick_x;

        frontLeftPower = (y + x + rx) ;
        backLeftPower = (y - x + rx) ;
        frontRightPower = (y - x - rx) ;
        backRightPower = (y + x - rx) ;


        robot.leftFront.setPower(frontLeftPower);
        robot.leftRear.setPower(backLeftPower);
        robot.rightFront.setPower(frontRightPower);
        robot.rightRear.setPower(backRightPower);
        //*telemetry
        double loop = System.nanoTime();
        double y = -robot.follower.getPose().getY() + RobotHardware.TARGET_GOAL.getY();
        double x = -robot.follower.getPose().getX() + RobotHardware.TARGET_GOAL.getX();
        double distance = Math.hypot(x, y);

        telemetry.addData("follower pose", robot.follower.getPose());
        telemetry.addData("x:", robot.follower.getPose().getX());
        telemetry.addData("y:", robot.follower.getPose().getY());
        telemetry.addData("heading:", robot.follower.getPose().getHeading());
        telemetry.addData("distance", distance);
//        telemetry.addData("hashVelocity", RobotHardware.hashVelocity);
//        telemetry.addData("hashAngle", RobotHardware.hashAngle);
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        //telemetry.addData("pp ", robot.follower.getPose());
        telemetry.addData("manual angle: ", manualAngle);
        telemetry.addData("manual velocity", manualVelocity);
        telemetry.addData("velocity", robot.TurretVelocitySubsystem.getVelocity());
        telemetry.addData("Targetvelocity", robot.TurretVelocitySubsystem.getTargetVelocity());
        telemetry.addData("turret heading", robot.Turret.getTurretHeading());
        telemetry.addData("Targetheading", robot.Turret.getTurretTargetHeading());
        telemetry.addData("lastAutoPose", RobotHardware.lastAutoPose);
        //telemetry.addData("isShootingFar: ", isShootingFar);
//        telemetry.addData("robot heading: ", robot.follower.getHeading());
//        telemetry.addData("desired heading: ", RobotHardware.desiredHeading);
//        telemetry.addData("ErrorHeading: ", errorHeading);

        loopTime = loop;
        telemetry.update();
        robot.clearBulkCache();
    }
}
