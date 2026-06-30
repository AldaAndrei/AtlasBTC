package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    private double targetVelocity;
    public static double requiredSpeed = 0;

    public static double F = 0.0003;
    public static double P = 0.01;
    public static double D = 0;
    public static double I = 0;
    private PIDFController launcherController;


    public enum LauncherState {
        IDLE,
        SHOOTING
    }
    LauncherState currentLauncherState = LauncherState.IDLE;


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


    }

    void updateLauncherState(){


        switch (currentLauncherState){
            case IDLE:
                masterMotor.setPower(0);
                followerMotor.setPower(0);
                break;

            case SHOOTING:
                double currentVel = getVelocity();

                // 2. CALCULATE Error
                double error = requiredSpeed - currentVel;

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

}
