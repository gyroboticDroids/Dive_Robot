package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TestMotors")
public class TestMotors extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    private DcMotor vertSlide1;
    private DcMotor vertSlide2;

    private DcMotor horizontalSlide;

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
                "\nTo move horizontal slides use gamepad 1 left stick x \n");

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
    }
}
