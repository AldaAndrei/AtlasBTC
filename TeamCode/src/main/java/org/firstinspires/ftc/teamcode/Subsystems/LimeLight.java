package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimeLight extends SubsystemBase {
    private Telemetry telemetry;
    private Limelight3A limelight;
    private IMU imu;
    Follower follower;

    private LLResult result;

    // Target data
    static double tx;
    static double ty;
    static double ta;

    // Robot Position
    static double robotCoordsX;
    static double robotCoordsY;
    static double robotCoordsZ;
    static int id;

    Pose llPose;

    // Automatic relocalization rate limiter
    private ElapsedTime relocalizationCooldown = new ElapsedTime();

    public enum LimelightMode {
        READ_PATTERN,
        TRACK_ARTIFACT,
        BASKET,
        PAUSE
    }
    private static LimelightMode currentMode;

    public LimeLight(HardwareMap hwMap, Follower follower, Telemetry telemetry) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        imu = hwMap.get(IMU.class, "imu");
        this.follower = follower;

        limelight.setPollRateHz(70);

        this.telemetry = telemetry;
    }

    public LimeLight(HardwareMap hwMap, Follower follower) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        imu = hwMap.get(IMU.class, "imu");
        this.follower = follower;

        limelight.setPollRateHz(70);
    }


}