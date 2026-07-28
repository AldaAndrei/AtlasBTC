package org.firstinspires.ftc.teamcode.Subsystems;


import static com.seattlesolvers.solverslib.purepursuit.PurePursuitUtil.angleWrap;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

@Config
public class TurretSubsystem extends SubsystemBase {
    private RobotHardware robot = RobotHardware.getInstance();

    double targetHeading;
    double power;
    double gearRatio = 5.75;
    double TicksPerRev = 145.6;
    public static boolean hasmap = false;
    public static double manualHoodAngle = 0;
    public static double manualVelocity = 0;

    double velocity;

    double target;

    @Override
    public void periodic() {
        robot.follower.update();


        robot.turretController.setPIDF(RobotHardware.P, RobotHardware.I, RobotHardware.D, RobotHardware.F);

        // Primary: FULL_PINPOINT odometry heading (same math as FULL_PINPOINT case)
        Pose mixedPose = robot.follower.getPose();
        double mixedFieldHeading = Math.atan2(
                RobotHardware.TARGET_GOAL.getY() - mixedPose.getY(),
                RobotHardware.TARGET_GOAL.getX() - mixedPose.getX());
        targetHeading = mixedFieldHeading - mixedPose.getHeading();


        //angle wrapping custom for mechanical limitation
        if(targetHeading>Math.toRadians(180)){targetHeading -= 2*Math.PI;}
        if(targetHeading<Math.toRadians(-180)){targetHeading += 2*Math.PI;}

        // pid controller handles antistrangulation byitself (no extra logic needed)
        double mixedError = targetHeading - getTurretHeading();


        power = robot.turretController.calculate(0, mixedError);
        robot.TurretMotor.setPower(power);


        double distance = Math.hypot(RobotHardware.TARGET_GOAL.getX() - mixedPose.getX() , RobotHardware.TARGET_GOAL.getY() - mixedPose.getY());

        if(!hasmap)
        {
            velocity = RobotHardware.getVelocity(distance);

            robot.TurretVelocitySubsystem.setTargetVelocity(velocity + 20);

            robot.TurretAngleServo.setPosition(RobotHardware.getHoodAngle(distance));
        }
        else
        {

            robot.TurretAngleServo.setPosition(manualHoodAngle);
            robot.TurretVelocitySubsystem.setTargetVelocity(manualVelocity);
        }
        robot.TurretVelocitySubsystem.update();
    }
    public double getTurretHeading() {
        double ticks = robot.TurretMotor.getCurrentPosition();
        // Formula: (Ticks / Total_Ticks_Per_Rev) * 2PI
        return (ticks / (TicksPerRev * gearRatio)) * 2 * Math.PI;
    }

    public double getTurretTargetHeading() {
        return targetHeading;
    }

    public boolean isNearSetPoint(){
        return Math.abs(angleWrap(targetHeading - getTurretHeading()))
                < Math.toRadians(15);
    }


}
