package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Intake extends SubsystemBase {

    // --- TUNING VARIABLES (Edit in FTC Dashboard) ---
    // F (Feedforward): Base power to hold speed. Start small (0.0001 - 0.0005)
    // P (Proportional): "Snap" power to fix errors.

    private double targetVelocity = 0.0;
    public static double servoPosDown = 0;
    public static double servoPosUp = 0.4;

    private DcMotorEx intakeMotor;
    private Servo intakeHold;


    /**
     * Defines the possible operational states for the Intake subsystem.
     */
    public enum IntakeState {
        INTAKE,
        SHOOT,
        REVERSE,
        IDLE
    }

    private IntakeState currentIntakeState = IntakeState.IDLE;

    /**
     * Constructs a new Intake subsystem.
     *
     * @param hwMap The hardware map from the OpMode.
     */
    public Intake(HardwareMap hwMap) {
        // 1. Hardware Mapping
        this.intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        this.intakeHold = hwMap.get(Servo.class, "intakeHold");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // 3. Float behavior (optional, allows free spin when 0 power) or BRAKE
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }
}
