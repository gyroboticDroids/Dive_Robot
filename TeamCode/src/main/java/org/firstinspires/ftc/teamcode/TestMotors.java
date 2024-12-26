package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.teleop.Outtake;

@TeleOp(name = "TestMotors")
public class TestMotors extends OpMode {
    private Hardware hardware;
    private Outtake outtake;

    float extensionControls = 0;

    @Override
    public void init()
    {
        hardware = new Hardware(hardwareMap);
        outtake = new Outtake(hardwareMap);
    }

    double horizontalPosition = 0;
    @Override
    public void loop()
    {
        //User info
        telemetry.addLine("Controls: \nA B X Y + gamepad 1 left stick y to move drive motors \nGamepad 1 right stick y for vertical slides " +
                "\nGamepad 1 left stick x to move horizontal slides \nGamepad 1 triggers for extension slides \n ");

        if(gamepad1.a)
        {
            hardware.frontLeft.setPower(gamepad1.left_stick_y);
        }
        else
        {
            hardware.frontLeft.setPower(0);
        }

        if(gamepad1.b)
        {
            hardware.frontRight.setPower(gamepad1.left_stick_y);
        }
        else
        {
            hardware.frontRight.setPower(0);
        }

        if(gamepad1.x)
        {
            hardware.rearLeft.setPower(gamepad1.left_stick_y);
        }
        else
        {
            hardware.rearLeft.setPower(0);
        }

        if(gamepad1.y)
        {
            hardware.rearRight.setPower(gamepad1.left_stick_y);
        }
        else
        {
            hardware.rearRight.setPower(0);
        }

        //hardware.vertSlide1.setPower(gamepad1.right_stick_y);
        //hardware.vertSlide2.setPower(-gamepad1.right_stick_y);


        horizontalPosition -= gamepad2.left_stick_y;

        horizontalPosition = MathFunctions.clamp(horizontalPosition, 0, Constants.HORIZONTAL_SLIDES_MAX);

        telemetry.addData("gamepad control", horizontalPosition);

        double error = horizontalPosition - hardware.horizontalSlide.getCurrentPosition();

        double motorPower = error * Constants.HORIZONTAL_SLIDES_P_GAIN;
        motorPower = Math.min(Math.max(motorPower, -0.6), 0.6);

        telemetry.addData("motor power", motorPower);

        hardware.horizontalSlide.setPower(motorPower);

        //hardware.horizontalSlide.setPower(gamepad1.left_stick_x);

        outtake.VertSlidesUpdate();
        outtake.VertSlidesManual(-gamepad1.left_stick_y);
        //extensionControls += (gamepad1.right_trigger - gamepad1.left_trigger) / 100;
        //extensionControls = Math.min(Math.max(extensionControls ,1), -1);

        //hardware.specimenExtension.setPosition(extensionControls);

        telemetry.addData("Horizontal slide pos", hardware.horizontalSlide.getCurrentPosition());
        telemetry.addData("Vertical slide pos", hardware.vertSlide1.getCurrentPosition());
        telemetry.addData("Extension slide pos", hardware.specimenExtension.getPosition());
        telemetry.addData("Horizontal slide power", hardware.horizontalSlide.getPower());
        telemetry.addData("Outtake servo pos", hardware.outtake.getPosition());

        telemetry.update();
    }
}
