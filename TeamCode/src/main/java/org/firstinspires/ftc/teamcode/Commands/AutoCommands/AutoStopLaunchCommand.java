package org.firstinspires.ftc.teamcode.Commands.AutoCommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.IntakeMotorCommand;
import org.firstinspires.ftc.teamcode.Commands.StopperServoCommand;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretVelocitySubsystem;

public class AutoStopLaunchCommand extends SequentialCommandGroup {
    public AutoStopLaunchCommand(TurretVelocitySubsystem launcher, TurretSubsystem turret){
        addCommands(
                new StopperServoCommand(RobotHardware.StopperServoClosed),
                new IntakeMotorCommand(0)
        );
    }
}
