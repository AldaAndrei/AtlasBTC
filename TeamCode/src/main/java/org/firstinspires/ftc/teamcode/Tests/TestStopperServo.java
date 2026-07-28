package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

@TeleOp(group = "test")
public class TestStopperServo extends LinearOpMode {
    Servo StopperServo;
    Servo StopperServo2;
    double pozitie = RobotHardware.StopperServoOpen;

    @Override
    public void runOpMode() {
        StopperServo = hardwareMap.get(Servo.class, "StopperServo");
        StopperServo2 = hardwareMap.get(Servo.class, "StopperServo2");

        waitForStart();
        while (opModeIsActive()) {
            pozitie += 0.0005 * gamepad1.right_stick_y;
            pozitie = Range.clip(pozitie, 0, 1);
            StopperServo.setPosition(pozitie);
            StopperServo2.setPosition(pozitie);
            telemetry.addData("servoPosition: ", StopperServo.getPosition());
            telemetry.addData("servoPosition2: ", StopperServo2.getPosition());
            telemetry.update();
        }
    }
}
