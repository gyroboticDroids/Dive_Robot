package org.firstinspires.ftc.teamcode;

import com.pedropathing.pathgen.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestMotors")
public class TestMotors extends OpMode {
    private static final double SENSITIVITY = 0.001;
    private Hardware hardware;

    private int state = 0;
    private boolean prevGp1Start;
    private boolean gpsAtRest = true;

    @Override
    public void init()
    {
        hardware = new Hardware(hardwareMap);
    }

    @Override
    public void start()
    {
        //outtake
        hardware.outtakeExtension.setPosition(0.5);
        hardware.outtakePivot.setPosition(0.5);
        hardware.outtakeWrist.setPosition(0.5);
        hardware.outtakeClaw.setPosition(0.5);

        //intake
        hardware.intakePivot.setPosition(0.5);

        //hang
        hardware.hangRight.setPosition(0.5);
        hardware.hangLeft.setPosition(0.5);
    }

    @Override
    public void loop()
    {
        telemetry.addLine("gpad 1 start to switch motors moved");
        telemetry.addLine();

        if(gamepad1.start && !prevGp1Start && gpsAtRest) {
            state++;

            if(state > 3) {
                state = 0;
            }
        }

        switch (state)
        {
            case 0:
                telemetry.addLine("drive");

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

                telemetry.addData("front left power (g1 left stick y + a)", hardware.frontLeft.getPower());
                telemetry.addData("front right power (g1 left stick y + b)", hardware.frontRight.getPower());
                telemetry.addData("rear left power (g1 left stick y + x)", hardware.rearLeft.getPower());
                telemetry.addData("rear right power (g1 left stick y + y)", hardware.rearRight.getPower());

                telemetry.addData("direction", Math.toDegrees(hardware.pinpointDriver.getHeading()));
                break;

            case 1:
                telemetry.addLine("outtake");

                hardware.outtakeSlide1.setPower(gamepad1.left_stick_y);
                hardware.outtakeSlide2.setPower(gamepad1.right_stick_y);

                hardware.outtakePivot.setPosition(MathFunctions.clamp(hardware.outtakePivot.getPosition() + gamepad2.left_stick_y * SENSITIVITY, 0, 1));
                hardware.outtakeWrist.setPosition(MathFunctions.clamp(hardware.outtakeWrist.getPosition() + gamepad2.right_stick_y * SENSITIVITY, 0, 1));
                hardware.outtakeExtension.setPosition(MathFunctions.clamp(hardware.outtakeExtension.getPosition() + (gamepad2.right_trigger - gamepad2.left_trigger) * SENSITIVITY, 0, 1));
                hardware.outtakeClaw.setPosition(MathFunctions.clamp(hardware.outtakeClaw.getPosition() + (gamepad1.right_trigger - gamepad1.left_trigger) * SENSITIVITY, 0, 1));

                telemetry.addData("slide 1 power (g1 left stick y)", hardware.outtakeSlide1.getPower());
                telemetry.addData("slide 2 power (g1 right stick y)", hardware.outtakeSlide2.getPower());
                telemetry.addData("slide 1 position", hardware.outtakeSlide1.getCurrentPosition());

                telemetry.addData("pivot position (g2 left stick y)", hardware.outtakePivot.getPosition());
                telemetry.addData("wrist position (g2 right stick y)", hardware.outtakeWrist.getPosition());
                telemetry.addData("extension position (g2 triggers)", hardware.outtakeExtension.getPosition());
                telemetry.addData("claw position (g1 triggers)", hardware.outtakeClaw.getPosition());
                break;

            case 2:
                telemetry.addLine("intake");

                hardware.intakeSlide.setPower(gamepad1.left_stick_y);

                hardware.intakePivot.setPosition(MathFunctions.clamp(hardware.intakePivot.getPosition() + gamepad1.right_stick_y * SENSITIVITY, 0, 1));
                hardware.intakeLeft.setPower(gamepad2.left_stick_y);
                hardware.intakeRight.setPower(gamepad2.right_stick_y);

                telemetry.addData("slide power (g1 left stick y)", hardware.intakeSlide.getPower());
                telemetry.addData("slide position", hardware.intakeSlide.getCurrentPosition());

                telemetry.addData("pivot position (g1 right stick y)", hardware.intakePivot.getPosition());
                telemetry.addData("intake left power (g2 left stick y)", hardware.intakeLeft.getPower());
                telemetry.addData("intake right power (g2 right stick y)", hardware.intakeRight.getPower());
                break;

            case 3:
                telemetry.addLine("hang");

                hardware.hangLeft.setPosition(MathFunctions.clamp(hardware.hangLeft.getPosition() + gamepad1.left_stick_y * SENSITIVITY, 0, 1));
                hardware.hangRight.setPosition(MathFunctions.clamp(hardware.hangRight.getPosition() + gamepad1.right_stick_y * SENSITIVITY, 0, 1));

                telemetry.addData("hang left position (g1 left stick y)", hardware.hangLeft.getPosition());
                telemetry.addData("hang right position (g1 right stick y)", hardware.hangRight.getPosition());
                break;
        }

        telemetry.update();

        prevGp1Start = gamepad1.start;
        gpsAtRest = gamepad1.atRest() || gamepad2.atRest();
    }
}
