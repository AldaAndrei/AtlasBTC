package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class SavePose extends InstantCommand{
    public SavePose(Pose pose){
        super(
                ()-> {
                    RobotHardware.END_POSE = pose;
                }
        );
    }

}
