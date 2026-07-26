package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDController {
    double p, i, d, maxPower;
    double lastError;
    double integralSum;
    double derivative;
    double error;
    double target;
    double power;
    ElapsedTime timer = new ElapsedTime();

    public PIDController(double p, double i, double d, double maxPower) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.maxPower = maxPower;
        timer.reset();
    }

    public double calculate(double currentPose){

        error = target - currentPose;

        derivative = (error - lastError) / timer.seconds();

        integralSum = integralSum + (error * timer.seconds()); //poate o sa trebuiasca clipuit

        power = Range.clip((p * error) + (i * integralSum) + (d * derivative),-maxPower, maxPower);

        lastError = error;

        timer.reset();

        return power;
    }

    public double getPower() {
        return power;
    }

    public void setP(double p) {
        this.p = p;
    }

    public void setI(double i) {
        this.i = i;
    }

    public void setD(double d) {
        this.d = d;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void setTarget(double target) {
        this.target = target;
    }
}
