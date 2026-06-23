package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Transfer extends SubsystemBase {

    // --- TUNING VARIABLES (Edit in FTC Dashboard) ---
    // F (Feedforward): Base power to hold speed. Start small (0.0001 - 0.0005)
    // P (Proportional): "Snap" power to fix errors.


    public static double servoPosDown = 0;
    public static double servoPosUp = 0.4;

    private DcMotorEx TransferMotor;
    private Servo TransferServo;


    /**
     * Defines the possible operational states for the Transfer subsystem.
     */
    public enum TransferState {
        FORWARD,
        REVERSE,
        IDLE
    }

    private TransferState currentTransferState = TransferState.IDLE;

    /**
     * Constructs a new Transfer subsystem.
     *
     * @param hwMap The hardware map from the OpMode.
     */
    public Transfer(HardwareMap hwMap) {
        // 1. Hardware Mapping
        this.TransferMotor = hwMap.get(DcMotorEx.class, "transferMotor");
        this.TransferServo = hwMap.get(Servo.class, "transferServo");

        TransferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // 3. Float behavior (optional, allows free spin when 0 power) or BRAKE
        TransferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }
}
