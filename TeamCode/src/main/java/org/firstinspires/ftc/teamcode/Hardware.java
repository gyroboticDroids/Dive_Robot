package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.GoBildaPinpointDriver;

public class Hardware {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor rearLeft;
    public DcMotor rearRight;

    public DcMotor vertSlide1;
    public DcMotor vertSlide2;

    public DcMotor horizontalSlide;

    public Servo specimenExtension;

    public Servo pivot;
    public Servo wrist;

    public Servo intake;
    public Servo outtake;

    public CRServo hangLeft;
    public CRServo hangRight;

    public GoBildaPinpointDriver pinpointDriver;

    public Hardware(HardwareMap hardwareMap)
    {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        vertSlide1 = hardwareMap.get(DcMotor.class, "vertSlide1");
        vertSlide2 = hardwareMap.get(DcMotor.class, "vertSlide2");

        horizontalSlide = hardwareMap.get(DcMotor.class, "horizontalSlide");

        specimenExtension = hardwareMap.get(Servo.class, "extension");

        pivot = hardwareMap.get(Servo.class, "pivot");
        wrist = hardwareMap.get(Servo.class, "wrist");

        outtake = hardwareMap.get(Servo.class, "outake");
        intake = hardwareMap.get(Servo.class, "intake");

        vertSlide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vertSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertSlide2.setDirection(DcMotor.Direction.REVERSE);

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
    }
}
