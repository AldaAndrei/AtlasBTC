package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;


@Config
public class Launcher extends SubsystemBase {
    // "Master" = motorDreapta (The one with the Encoder Cable plugged in)
    private DcMotorEx masterMotor;

    // "Follower" = motorStanga (Encoder ignored, just follows power)
    private DcMotorEx followerMotor;

    Follower follower;
    private Servo hoodServo;

    public static double F = 0.0003;
    public static double P = 0.01;
    public static double D = 0;
    public static double I = 0;
    private PIDFController launcherController;

    // --- SHOOT-ON-THE-FLY CONSTANTS ---
    // GoBILDA 5000 series 6000RPM, 28 ticks/rev, 72mm flywheel, 0.45 transfer efficiency
    // K_LAUNCHER = 0.45 × π × 0.072m × 39.37in/m / 28 ticks/rev ≈ 0.1431 (in/s per tick/s) transforma getvelocity in viteza mingi de iesire totala
    public static double K_LAUNCHER = 0.1431;
    public static double currentHoodAngleDeg = 47.0;  // updated every loop, read by Turret
    public static double baseTargetVelocity  = 0.0;

    Pose goalPose;

    public enum LauncherState {
        IDLE,
        SHOOTING
    }
    LauncherState currentLauncherState = LauncherState.IDLE;
    public void setCurrentLauncherState(LauncherState currentLauncherState) {
        if(this.currentLauncherState != currentLauncherState){
            launcherController.reset();
        }

        this.currentLauncherState = currentLauncherState;
    }


    public Launcher(HardwareMap hwMap, Follower flwr) {
        // 1. Hardware Mapping - HERE is where we define the Master
        masterMotor = hwMap.get(DcMotorEx.class, "motorDreapta"); // MUST have encoder cable
        followerMotor = hwMap.get(DcMotorEx.class, "motorStanga");// Encoder optional/ignored
        hoodServo = hwMap.get(Servo.class, "hoodServo");


        follower = flwr;


        // 3. Set to RUN_WITHOUT_ENCODER
        // This is CRITICAL. It tells the internal REV hub "Don't use your built-in PID,
        // let me handle the math myself in the periodic() function."
        masterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        followerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 4. Float behavior for smoother deceleration
        masterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        followerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 5. Direction Setup
        // Check this physically! Usually, flywheels spin opposite ways to shoot forward.
        // If the robot shoots backward, remove this REVERSE or move it to followerMotor.
        masterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherController = new PIDFController(P, I, D, F);

        goalPose = (alliance == Alliance.RED) ? redGoalPose : blueGoalPose;
    }

    void updateLauncherState(){


        switch (currentLauncherState){
            case IDLE:
                masterMotor.setPower(0);
                followerMotor.setPower(0);
                break;

            case SHOOTING:
                double currentVel = getVelocity();
                launcherController.setSetPoint(requiredSpeed);

                // 3. CALCULATE Power (PF Controller)
                // Feedforward (F): Base power to maintain target
                // Proportional (P): Correction power based on error
                double power = launcherController.calculate(currentVel);

                // 5. APPLY the SAME calculated power to BOTH motors
                // This ensures they stay synced, driven by motorDreapta's encoder data.
                masterMotor.setPower(power);
                followerMotor.setPower(power);

                break;
        }
    }

    public double getTargetVelocity(){
        return targetVelocity;
    }
    public double getVelocity() {
        return masterMotor.getVelocity();
    }
    public void setHoodPose(double pos) {
        hoodServo.setPosition(pos);
    }
    public void stop() {
        setCurrentLauncherState(LauncherState.IDLE);
    }
    public boolean isVelocityReached() {
        return getVelocity() > requiredSpeed - 50;
    }
    public double getDistance(){
        return Math.sqrt(Math.pow(follower.getPose().getX() - goalPose.getX(),2) + Math.pow(follower.getPose().getY() - goalPose.getY(),2));
    }

    public void init(){
        setCurrentLauncherState(LauncherState.IDLE);
        setHoodPose(0);
    }
    @Override
    public void periodic() {
        updateLauncherState();


        // Base (stationary) velocity — also read by Turret for SOF angle+speed compensation
        //baseTargetVelocity = Math.pow(getDistance(), 0.4706919) * 189.0741;  refacem testele cand avem robotul (https://mycurvefit.com/)
        targetVelocity = baseTargetVelocity;

    }

}
