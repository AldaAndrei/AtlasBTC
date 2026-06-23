package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

@Config
public class Turret extends SubsystemBase {

    private Servo Servo1;
    private Servo Servo2;
    private Servo Servo3;


    public Pose goalPose;
    public Pose2d targetGoalPose;



    public enum TurretState {
        IDLE,
        FULL_LIMELIGHT,
        FULL_PINPOINT,
        MIXED,
        SHOOT_ON_THE_FLY,
    }

    private static TurretState currentTurretState = TurretState.IDLE;

    public Turret(HardwareMap hwMap, Follower flwr) {
        Servo1 = hwMap.get(Servo.class, "servo1");
        Servo2 = hwMap.get(Servo.class, "servo2");
        Servo3 = hwMap.get(Servo.class, "servo3");


    }
}