package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestMotors")
public class TestMotors extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    private DcMotor vertSlide1;
    private DcMotor vertSlide2;

    private DcMotor horizontalSlide;

    private Servo specimenExtension;

    private Servo intake;
    private Servo outake;

    private CRServo hangLeft;
    private CRServo hangRight;

    float extensionControls = 0;

    @Override
    public void init()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeftMotor");
        rearRight = hardwareMap.get(DcMotor.class, "rearRightMotor");

        vertSlide1 = hardwareMap.get(DcMotor.class, "vertSlide1");
        vertSlide2 = hardwareMap.get(DcMotor.class, "vertSlide2");

        horizontalSlide = hardwareMap.get(DcMotor.class, "horizontalSlide");

        specimenExtension = hardwareMap.get(Servo.class, "extension");

        outake = hardwareMap.get(Servo.class, "outake");
        intake = hardwareMap.get(Servo.class, "intake");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop()
    {
        //User info
        telemetry.addLine("Controls: \nA B X Y + gamepad 1 left stick y to move drive motors \nGamepad 1 right stick y for vertical slides " +
                "\nGamepad 1 left stick x to move horizontal slides \nGamepad 1 triggers for extension slides \n ");

        if(gamepad1.a)
        {
            frontLeft.setPower(gamepad1.left_stick_y);
        }
        else
        {
            frontLeft.setPower(0);
        }

        if(gamepad1.b)
        {
            frontRight.setPower(gamepad1.left_stick_y);
        }
        else
        {
            frontRight.setPower(0);
        }

        if(gamepad1.x)
        {
            rearLeft.setPower(gamepad1.left_stick_y);
        }
        else
        {
            rearLeft.setPower(0);
        }

        if(gamepad1.y)
        {
            rearRight.setPower(gamepad1.left_stick_y);
        }
        else
        {
            rearRight.setPower(0);
        }

        vertSlide1.setPower(gamepad1.right_stick_y);
        vertSlide2.setPower(-gamepad1.right_stick_y);

        horizontalSlide.setPower(gamepad1.left_stick_x);

        extensionControls += (gamepad1.right_trigger - gamepad1.left_trigger) / 100;
        extensionControls = Math.min(Math.max(extensionControls ,1), -1);

        specimenExtension.setPosition(extensionControls);

        telemetry.addData("Horizontal slide pos", horizontalSlide.getCurrentPosition());
        telemetry.addData("Vertical slide pos", vertSlide2.getCurrentPosition());
        telemetry.addData("Extension slide pos", specimenExtension.getPosition());
        telemetry.addData("Intake servo pos", intake.getPosition());
        telemetry.addData("Outake servo pos", outake.getPosition());

        telemetry.update();
    }
}
