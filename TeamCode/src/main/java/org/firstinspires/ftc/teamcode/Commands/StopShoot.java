package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class StopShoot extends SequentialCommandGroup {
    public StopShoot(){
        super(
                new IntakeMotorCommand(0),
                new StopperServoCommand(RobotHardware.StopperServoClosed)
        );
    }
}
