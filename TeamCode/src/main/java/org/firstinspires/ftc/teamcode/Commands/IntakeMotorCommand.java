package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class IntakeMotorCommand extends InstantCommand {
    public IntakeMotorCommand(double target){
        super(
                ()-> RobotHardware.getInstance().IntakeMotor.setPower(target)
        );
    }
}
