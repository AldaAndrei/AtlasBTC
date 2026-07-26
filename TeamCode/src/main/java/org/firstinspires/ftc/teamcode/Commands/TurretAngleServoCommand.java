package org.firstinspires.ftc.teamcode.Commands;


import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class TurretAngleServoCommand extends InstantCommand {
    public TurretAngleServoCommand(double target){
        super(
                ()-> RobotHardware.getInstance().TurretAngleServo.setPosition(target)
        );
    }
}
