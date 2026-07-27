package org.firstinspires.ftc.teamcode.Commands.AutoCommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.IntakeMotorCommand;
import org.firstinspires.ftc.teamcode.Commands.StopperServoCommand;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretVelocitySubsystem;


/**
 * Shoots using MIXED turret state: odometry-primary heading (90%) with a small
 * limelight fine-trim on top (10%). No robot velocity compensation.
 *
 * Unlike ShootCommand / MixedAimCommand, this does NOT transition through
 * FULL_PINPOINT → FULL_LIMELIGHT. The turret stays in MIXED the entire time,
 * continuously blending limelight correction into the odometry heading.
 *
 * Waits for BOTH flywheel speed AND turret near-setpoint before opening the stopper,
 * giving the clearest shot possible without adding a hard limelight-lock gate.
 */
public class AutoMixedShootCommand extends SequentialCommandGroup {
    public AutoMixedShootCommand(TurretVelocitySubsystem launcher, TurretSubsystem turret) {
        addCommands(
                new WaitUntilCommand(() -> launcher.isReady() && turret.isNearSetPoint()),
                new StopperServoCommand(RobotHardware.StopperServoClosed),
                new IntakeMotorCommand(1)
        );
    }
}
