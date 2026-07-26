package org.firstinspires.ftc.teamcode.Subsystems;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class TurretSubsystem extends SubsystemBase {
    private RobotHardware robot = RobotHardware.getInstance();

    double targetHeading;
    double power;
    double gearRatio = 5.75;
    double TicksPerRev = 145.6;

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
        if(targetHeading>Math.toRadians(270)){targetHeading -= 2*Math.PI;}
        if(targetHeading<Math.toRadians(-90)){targetHeading += 2*Math.PI;}

        // pid controller handles antistrangulation byitself (no extra logic needed)
        double mixedError = targetHeading - getTurretHeading();


        power = robot.turretController.calculate(0, mixedError);
        robot.TurretMotor.setPower(power);


        double distance = Math.hypot( mixedPose.getX() , mixedPose.getY() );

        velocity = RobotHardware.getVelocity(distance);

        robot.TurretVelocitySubsystem.setTargetVelocity(velocity + 20);

        robot.TurretVelocitySubsystem.update();
    }
    public double getTurretHeading() {
        double ticks = robot.TurretMotor.getCurrentPosition();
        // Formula: (Ticks / Total_Ticks_Per_Rev) * 2PI
        return (ticks / (TicksPerRev * gearRatio)) * 2 * Math.PI;
    }

    double correctedHeadingDegrees(double heading){
        if (heading > 0) {
            heading -= 2 * Math.PI;
        } else if (heading < -2 * Math.PI) {
            heading += 2 * Math.PI;
        }
        return Math.toDegrees(heading);
    }

}
