package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    //Drive
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor rearLeft;
    public DcMotor rearRight;

    public GoBildaPinpointDriver pinpointDriver;

    //Outtake
    public DcMotor outtakeSlide1;
    public DcMotor outtakeSlide2;

    public Servo outtakeExtension;
    public Servo outtakePivot;
    public Servo outtakeWrist;
    public Servo outtakeClaw;

    //Intake
    public DcMotor intakeSlide;

    public Servo intakePivot;

    public CRServo intakeRight;
    public CRServo intakeLeft;

    public RevColorSensorV3 colorSensor;

    //Hang
    public Servo hangRight;
    public Servo hangLeft;

    public Hardware(HardwareMap hardwareMap)
    {
        //Drive
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpointDriver.setOffsets(-12.0, -60.0); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpointDriver.resetPosAndIMU();

        //Outtake
        outtakeSlide1 = hardwareMap.get(DcMotor.class, "vertSlide1");
        outtakeSlide2 = hardwareMap.get(DcMotor.class, "vertSlide2");
        outtakeExtension = hardwareMap.get(Servo.class, "extension");
        outtakePivot = hardwareMap.get(Servo.class, "pivot");
        outtakeWrist = hardwareMap.get(Servo.class, "wrist");
        outtakeClaw = hardwareMap.get(Servo.class, "claw");

        outtakeSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSlide1.setDirection(DcMotor.Direction.REVERSE);

        outtakeSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSlide2.setDirection(DcMotor.Direction.FORWARD);

        //Intake
        intakeSlide = hardwareMap.get(DcMotor.class, "horizontalSlide");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");

        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        //Hang
        hangRight = hardwareMap.get(Servo.class, "hangRight");
        hangLeft = hardwareMap.get(Servo.class, "hangLeft");
    }
}
