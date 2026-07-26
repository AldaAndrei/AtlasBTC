package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PDController {
    double p, d, maxPower;
    double lastError;
    double derivative;
    double error;
    double target;
    double power;
    ElapsedTime timer = new ElapsedTime();

    public PDController(double p, double d, double maxPower) {
        this.p = p;
        this.d = d;
        this.maxPower = maxPower;
        timer.reset();
    }

    public double calculate(double currentPose){

        error = target - currentPose;

        derivative = (error - lastError) / timer.seconds();

        power = Range.clip((p * error) + (d * derivative),-maxPower, maxPower);

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
