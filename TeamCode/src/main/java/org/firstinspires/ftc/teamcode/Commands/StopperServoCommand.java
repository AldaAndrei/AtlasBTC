package org.firstinspires.ftc.teamcode.Commands;


import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class StopperServoCommand extends SequentialCommandGroup {
    public StopperServoCommand(double target){
        super(
                new InstantCommand(()-> RobotHardware.getInstance().StopperServo.setPosition(target)),
                new InstantCommand(()-> RobotHardware.getInstance().StopperServo2.setPosition(target))
        );
    }
}
