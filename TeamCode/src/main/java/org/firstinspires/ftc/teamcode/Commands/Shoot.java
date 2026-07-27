package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Shoot extends SequentialCommandGroup {
    public Shoot(){
        super(
                new IntakeMotorCommand(1),
                new WaitCommand(300),
                new StopperServoCommand(RobotHardware.StopperServoOpen)
        );
    }
}
