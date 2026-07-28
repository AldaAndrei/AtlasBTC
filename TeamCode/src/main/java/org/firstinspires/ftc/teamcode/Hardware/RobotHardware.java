package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretVelocitySubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Config
public class RobotHardware {
    private static RobotHardware instance = null;
    public DcMotorEx rightRear, leftRear, leftFront, rightFront;
    List<LynxModule> allHubs;

    public TurretSubsystem Turret;
    public TurretVelocitySubsystem TurretVelocitySubsystem;



    //*math
    public static AngleUnit angleUnit = AngleUnit.RADIANS;
    public static DistanceUnit distanceUnit = DistanceUnit.INCH;

    //*location related
    public Follower follower;
    public GoBildaPinpointDriver pinpoint;

    //*turret related
    public DcMotorEx TurretMotor;
    public MotorEx LaunchSusMotor, LaunchJosMotor;
    public Servo TurretAngleServo;
    public static double TurretAngleServoInit = 0.1;
    public MotorEx.Encoder LaunchEncoder;

    //*stopper
    public Servo StopperServo;
    public Servo StopperServo2;
    public static double StopperServoOpen = 0.126;
    public static double StopperServoClosed = 0.4; //TODO schimba
    public static double StopperServoInit = StopperServoClosed;

    //*intake
    public Servo IntakeServo;
    public DcMotorEx IntakeMotor;
    public static double IntakeServoInit = 0.1;
    public static double IntakeServoUp = 0.1;
    public static double IntakeServoDown = 0.43;

    //*pid
    public com.seattlesolvers.solverslib.controller.PIDFController turretController;
    public static double P = 0.5, secondP = 1, I = 0, D = 0.05, F = 1.16;
    public static double velocityP = 0.0052, velocityI =0, velocityD = 0, velocityF = 0.00052;


    //*alliance
    public enum AllianceColor{
        BLUE,
        RED
    }
    public enum StartingPose{
        CLOSE,
        FAR
    }
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;
    public static StartingPose STARTING_POSE = StartingPose.CLOSE;

    public static double getVelocity(double distance) { //script felics
        if (distance < 35.11) {
            return 920;
        } else if (distance < 45.62) {
            return (int) (1110);
        } else if (distance < 55.11) {
            return (int) (1200);
        } else if (distance < 65.21) {
            return (int) (1250);
        } else if (distance < 75.26) {
            return (int) (1300);
        } else if (distance < 85.09) {
            return (int) (1360);
        } else if (distance < 95.88) {
            return (int) (1400);
        } else if (distance < 105.11) {
            return (int) (1500);
        } else if (distance < 115.02) {
            return (int) (1600);
        } else if (distance < 130) {
            return (int) (1700);
        } else {
            return 1850;
        }
        //return Math.pow(distance, 0.4760475) * 188.83;
        //trage mult prea tare cu formula veche, daca e ori lasam cu lookuptable sau fa testele dinnou ca nu dureaza mmult
    }

    public static double getHoodAngle(double distance) {
        if (distance < 35.11) {
            return 0.51;
        } else if (distance < 45.62) {
            return 0.51;
        } else if (distance < 55.11) {
            return 0.51;
        } else if (distance < 65.21) {
            return 0.28;
        } else if (distance < 75.26) {
            return 0.28;
        } else if (distance < 85.09) {
            return 0.25;
        } else if (distance < 95.88) {
            return 0.20;
        } else if (distance < 130) {
            return 0.15;
        } else {
            return 0.08;
        }
    }

    //*field related
    public static Pose GOAL_POSE_BLUE() {
        return new Pose(3.5, 144 - 3.5, 0);
    }
    public static Pose GOAL_POSE_RED() {
        return new Pose(144 - 3.5, 144 - 3.5, 0);
    }
    public static Pose RED_RESET_CLOSE() {return new Pose(126,84, 0);}
    public static Pose BLUE_RESET_CLOSE() {return new Pose(25, 84, Math.PI);}//133 6 colt // 16.5 86
    public static Pose RED_RESET_FAR() {return new Pose(77,18.5, 0);}
    public static Pose BLUE_RESET_FAR() {return new Pose(59, 16.5, Math.PI);}//70 18 colt // 16.5 86
    public static Pose RED_RESET_HUMAN() {return new Pose(9,32.5, Math.PI);}
    public static Pose BLUE_RESET_HUMAN() {return new Pose(131, 29, 0);}//70 18 colt // 16.5 86
    public static Pose BLUE_START() {return new Pose(27, 130, Math.toRadians(144));}
    public static Pose BLUE_START_FAR() {return new Pose(60, 8.5, Math.toRadians(180));}
    public static Pose RED_START_FAR() {return new Pose(144 - 60, 8.5, Math.toRadians(180));}


    public static Pose RED_START() {
        return new Pose(144 - 27, 130, Math.toRadians(180-144));
    }
    public static Pose TARGET_GOAL = new Pose(0, 144, 0);
    public static Pose END_POSE = new Pose(33, 138, Math.toRadians(180)); //BLUE
    public static Pose ResetPose;
    public static Pose ResetFarPose;
    public static Pose ResetHumanPose;
    public static double SCORE_HEIGHT = 26; //inch
    public static double SCORE_ANGLE = Math.toRadians(-30);

    //*led related
    public Servo ColorChannel;
    public static double off = 0;
    public static double red = 0.280;
    public static double orange = 0.333;
    public static double yellow = 0.338;
    public static double sage = 0.444;
    public static double green = 0.500;
    public static double azure = 0.555;
    public static double blue = 0.611;
    public static double indigo = 0.666;
    public static double violet = 0.720;
    public static double white = 1;

    //*auto related
    public boolean isAuto;

    //*custom methods
    public static RobotHardware getInstance() {
        if(instance == null){
            instance = new RobotHardware();
        }
        return instance;
    }
    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
    public void updateAlliance(AllianceColor allianceColor){
        ALLIANCE_COLOR = allianceColor;
        if (ALLIANCE_COLOR == AllianceColor.BLUE) {
            TARGET_GOAL = GOAL_POSE_BLUE();
            ColorChannel.setPosition(blue);
            ResetPose = BLUE_RESET_CLOSE();
            ResetFarPose = BLUE_RESET_FAR();
            ResetHumanPose = BLUE_RESET_HUMAN();
        } else {
            TARGET_GOAL = GOAL_POSE_RED();
            ColorChannel.setPosition(red);
            ResetPose = RED_RESET_CLOSE();
            ResetFarPose = RED_RESET_FAR();
            ResetHumanPose = RED_RESET_HUMAN();
        }
    }


    public void init(HardwareMap hardwareMap, boolean isAuto){
        this.isAuto = isAuto;

        allHubs = hardwareMap.getAll(LynxModule.class);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setOffsets(-5.2, 0,DistanceUnit.INCH);//TODO 2.7
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        TurretMotor = hardwareMap.get(DcMotorEx.class,"TurretMotor");
        TurretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        if(isAuto)
        {
            TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        TurretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        TurretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        LaunchJosMotor = new MotorEx(hardwareMap, "LaunchDreaptaMotor");
        LaunchJosMotor.setRunMode(Motor.RunMode.RawPower);
        LaunchJosMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        LaunchSusMotor = new MotorEx(hardwareMap, "LaunchStangaMotor");
        LaunchSusMotor.setRunMode(Motor.RunMode.RawPower);
        LaunchSusMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        LaunchJosMotor.setInverted(true);

        LaunchEncoder = new MotorEx(hardwareMap, "LaunchStangaMotor").encoder;

        TurretAngleServo = hardwareMap.get(Servo.class,"TurretAngleServo");
        StopperServo = hardwareMap.get(Servo.class,"StopperServo");
        StopperServo2 = hardwareMap.get(Servo.class,"StopperServo2");



        rightRear = hardwareMap.get(DcMotorEx.class,"backRight");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear = hardwareMap.get(DcMotorEx.class,"backLeft");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront = hardwareMap.get(DcMotorEx.class,"frontLeft");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.get(DcMotorEx.class,"frontRight");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        //leftRear.setDirection(DcMotorEx.Direction.REVERSE);


        IntakeMotor = hardwareMap.get(DcMotorEx.class,"IntakeMotor");
        IntakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        IntakeServo = hardwareMap.get(Servo.class,"IntakeServo");

        ColorChannel = hardwareMap.get(Servo.class,"ColorChannel");

        follower = Constants.createFollower(hardwareMap);

        Turret = new TurretSubsystem();
        TurretVelocitySubsystem = new TurretVelocitySubsystem();

        turretController = new PIDFController(P, I, D, F);

        if (ALLIANCE_COLOR == AllianceColor.BLUE) {
            TARGET_GOAL = GOAL_POSE_BLUE();
            ColorChannel.setPosition(blue);
            ResetPose = BLUE_RESET_CLOSE();
            ResetFarPose = BLUE_RESET_FAR();
            ResetHumanPose = BLUE_RESET_HUMAN();
        } else {
            TARGET_GOAL = GOAL_POSE_RED();
            ColorChannel.setPosition(red);
            ResetPose = RED_RESET_CLOSE();
            ResetFarPose = RED_RESET_FAR();
            ResetHumanPose = RED_RESET_HUMAN();
        }

        if(!isAuto) {
            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
            follower.setStartingPose(new Pose(END_POSE.getX(), END_POSE.getY(), END_POSE.getHeading()));
        }

    }
}
