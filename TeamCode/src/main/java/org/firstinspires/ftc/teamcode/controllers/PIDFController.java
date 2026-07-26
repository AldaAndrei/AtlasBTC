package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDFController {
    double p, i, d, f, maxPower;
    double lastError;
    double integralSum;
    double derivative;
    double error;
    double target;
    double power;
    double lastPower;
    double minIntegral, maxIntegral;
    double THRESHOLD = 0;
    double powerTHRESHOLD = 0;
    boolean isPowerTHRESHOLD = false;
    ElapsedTime timerPowerTHRESHOLD = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();



    public PIDFController(double p, double i, double d, double f, double maxPower) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.maxPower = maxPower;
        this.minIntegral = -1;
        this.maxIntegral = 1;
        this.lastPower = 0;
        timer.reset();
    }

    public double calculate(double currentPose){

        error = target - currentPose;

        derivative = (error - lastError) / timer.seconds();

        integralSum = integralSum + (error * timer.seconds()); //poate o sa trebuiasca clipuit

        integralSum = integralSum < minIntegral ? minIntegral : Math.min(maxIntegral, integralSum);

        power = Range.clip((p * error) + (i * integralSum) + (d * derivative) + (f * Math.signum(error)) ,-maxPower, maxPower);

        if (Math.abs(error) <= THRESHOLD){
            power = 0;
        }

        //* pentru tureta, limitam schimbarea powerului ca sa nu sara beltul
        if (isPowerTHRESHOLD && timerPowerTHRESHOLD.milliseconds() > 20 && Math.abs(power - lastPower) > powerTHRESHOLD) {
            power = Range.clip(power, lastPower - powerTHRESHOLD/2, lastPower + powerTHRESHOLD/2);
            timerPowerTHRESHOLD.reset();
        }

        lastError = error;

        lastPower = power;

        timer.reset();

        return power;
    }

    public void setIntegrationBounds(double integralMin, double integralMax) {
        minIntegral = integralMin;
        maxIntegral = integralMax;
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
    public void setF(double f){this.f = f;}

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void setTarget(double target) {
        this.target = target;
    }
    public void setTHRESHOLD(double t){
        this.THRESHOLD = t;
    }
    public void setPowerTHRESHOLD(double t){
        this.powerTHRESHOLD = t;
        isPowerTHRESHOLD = true;
    }
}
