package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    public void ConfigureHardware(@NonNull HardwareMap hardwareMap)
        {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
            rearLeft = hardwareMap.get(DcMotor.class, "rearLeftMotor");
            rearRight = hardwareMap.get(DcMotor.class, "rearRightMotor");

            vertSlide1 = hardwareMap.get(DcMotor.class, "vertSlide1");
            vertSlide2 = hardwareMap.get(DcMotor.class, "vertSlide2");

            horizontalSlide = hardwareMap.get(DcMotor.class, "horizontalSlide");

            specimenExtension = hardwareMap.get(Servo.class, "extension");

            pivot = hardwareMap.get(Servo.class, "pivot");
            wrist = hardwareMap.get(Servo.class, "wrist");

            outtake = hardwareMap.get(Servo.class, "outake");
            intake = hardwareMap.get(Servo.class, "intake");

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
