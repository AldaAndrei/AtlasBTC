package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class SavePose extends CommandBase {
    private final Follower follower;

    public SavePose(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void initialize() {
        // This executes when the command actually runs at the end of your sequence
        RobotHardware.lastAutoPose = follower.getPose();
    }

    @Override
    public boolean isFinished() {
        return true; // Ends immediately after executing initialize()
    }
}
