package org.firstinspires.ftc.teamcode.Commands;


import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class ColorChannelCommand extends InstantCommand {
    public ColorChannelCommand(double target){
        super(
                ()-> RobotHardware.getInstance().ColorChannel.setPosition(target)
        );
    }
}
