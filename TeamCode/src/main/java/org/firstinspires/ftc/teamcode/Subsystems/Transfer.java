package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Globals.targetVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Transfer extends SubsystemBase {

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

    TransferState currentTransferState = TransferState.IDLE;

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

    void updateTransferState(){
        switch (currentTransferState){
            case IDLE:
                TransferMotor.setPower(0);
                break;

            case FORWARD:
                TransferMotor.setPower(0.9);
                break;

            case REVERSE:
                TransferMotor.setPower(-0.9);
                break;
        }
    }
    public void setTransferState(TransferState TransferState){this.currentTransferState = TransferState;}
    public void init(){
        setTransferState(TransferState.IDLE);
    }

}
