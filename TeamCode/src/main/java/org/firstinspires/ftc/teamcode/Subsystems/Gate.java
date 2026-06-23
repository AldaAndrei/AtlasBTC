package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Gate extends SubsystemBase {


    public static double servoPosDown = 0;
    public static double servoPosUp = 0.4;

    private Servo Gate;


    /**
     * Defines the possible operational states for the Gate subsystem.
     */
    public enum IntakeState {
        GATE,
        IDLE
    }

    private IntakeState currentGateState = IntakeState.IDLE;

    /**
     * Constructs a new Gate subsystem.
     *
     * @param hwMap The hardware map from the OpMode.
     */
    public Gate(HardwareMap hwMap) {
        // 1. Hardware Mapping
        this.Gate = hwMap.get(Servo.class, "gate");


    }
}
