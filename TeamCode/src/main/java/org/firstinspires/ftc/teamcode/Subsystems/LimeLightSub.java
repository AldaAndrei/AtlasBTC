package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import static org.firstinspires.ftc.teamcode.Globals.*;

import java.lang.reflect.Modifier;
import java.util.List;

public class LimeLightSub extends SubsystemBase {
    private Limelight3A limelight;
    private IMU imu;
    Follower follower;

    private LLResult result;

    // Robot Position
    static double robotCoordsX;
    static double robotCoordsY;
    static double robotCoordsZ;
    static int id;
    Pose llPose;
    public enum LimelightMode {
        TRACK_ARTIFACT,
        PAUSE
    }
    private static LimelightMode currentMode;

    public LimeLightSub(HardwareMap hwMap, Follower follower) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        imu = hwMap.get(IMU.class, "imu");
        this.follower = follower;

        limelight.setPollRateHz(70);
    }

    public void init() {
        setMode(LimelightMode.PAUSE);
    }

    public void setMode(LimelightMode mode) {
        currentMode = mode;
        switch (currentMode) {
            case TRACK_ARTIFACT:
                limelight.pipelineSwitch(3);
                limelight.start();
                break;
            case PAUSE:
                limelight.pause();
                lltx = 0;
                llty = 0;
                llta = 0;
                break;
        }
    }

    /**
     * Run periodically by the CommandScheduler
     */
    @Override
    public void periodic() {


    }
}