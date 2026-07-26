package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOp.Tele0p;


@Config
public class TurretVelocitySubsystem extends SubsystemBase {
    private RobotHardware robot = RobotHardware.getInstance();
    private double targetVelocity = 0;
    public static double P = RobotHardware.velocityP;
    public static double I = RobotHardware.velocityI;
    public static double D = RobotHardware.velocityD;
    public static double F = RobotHardware.velocityF;
    private double power = 0;
    private PIDFController controller = new PIDFController(P, I, D, F);

    public void update() {
        controller.setP(RobotHardware.velocityP);
        controller.setI(RobotHardware.velocityI);
        controller.setD(RobotHardware.velocityD);
        controller.setF(RobotHardware.velocityF);

        controller.setSetPoint(targetVelocity + Tele0p.manualVelocity);

        power = controller.calculate(robot.LaunchEncoder.getCorrectedVelocity());

        setPower(-power);
    }

    public void setPower(double power) {
        this.power = power;
        robot.LaunchJosMotor.set(power);
        robot.LaunchSusMotor.set(power);
    }

    public void setTargetVelocity(double targetVelocity){this.targetVelocity = targetVelocity;}
    public double getTargetVelocity() { return targetVelocity; }
    public double getVelocity() { return robot.LaunchEncoder.getRawVelocity(); }
    public boolean isReady() { return Math.abs((targetVelocity + Tele0p.manualVelocity) - robot.LaunchEncoder.getRawVelocity()) <= 60;}

}
