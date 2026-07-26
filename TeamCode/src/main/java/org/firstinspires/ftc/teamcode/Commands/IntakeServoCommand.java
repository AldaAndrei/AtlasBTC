package org.firstinspires.ftc.teamcode.Commands;


import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class IntakeServoCommand extends InstantCommand {
    public IntakeServoCommand(double target){
        super(
                ()-> RobotHardware.getInstance().IntakeServo.setPosition(target)
        );
    }
}
