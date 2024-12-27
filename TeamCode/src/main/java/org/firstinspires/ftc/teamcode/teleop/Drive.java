package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

public class Drive {
    private final Hardware hardware;
    private final Gamepad gamepad;

    double y;
    double x;
    double rx;

    boolean resetHeading;
    double botHeading;
    double targetHeading = 0;

    double turnPower;

    double turnOffset = 0;

    public boolean driveBack = false;

    public Drive(HardwareMap hardwareMap, Gamepad gamepad1)
    {
        hardware = new Hardware(hardwareMap);
        gamepad = gamepad1;
    }

    public void Update()
    {
        hardware.pinpointDriver.update();

        if(driveBack)
        {
            hardware.frontLeft.setPower(Constants.DRIVE_BACK_POWER);
            hardware.rearLeft.setPower(Constants.DRIVE_BACK_POWER);
            hardware.frontRight.setPower(Constants.DRIVE_BACK_POWER);
            hardware.rearRight.setPower(Constants.DRIVE_BACK_POWER);

            return;
        }

        botHeading = hardware.pinpointDriver.getHeading() - turnOffset;

        Input(gamepad);
        Movement();
    }

    private void Input(Gamepad gpad)
    {
        double multiplier = (Math.abs(gpad.right_stick_x) + Math.abs(gpad.right_stick_y) > 0)? Constants.SLOW_SPEED_MULTIPLIER:Constants.DRIVE_SPEED_MULTIPLIER;

        y = gpad.left_stick_y * multiplier;
        x = -gpad.left_stick_x * multiplier;
        rx = (gpad.left_trigger - gpad.right_trigger) * multiplier;
        resetHeading = gpad.back;

        targetHeading = (gpad.y)? 0:(gpad.x)? 90: (gpad.a)? -45: (gpad.b)? -90:targetHeading;

        targetHeading += rx;
    }

    private void Movement()
    {
        if (resetHeading)
        {
            turnOffset = hardware.pinpointDriver.getHeading();
            targetHeading = 0;
        }

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        AutoTurn();

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnPower), 1);
        double frontLeftPower = (rotY + rotX + turnPower) / denominator;
        double backLeftPower = (rotY - rotX + turnPower) / denominator;
        double frontRightPower = (rotY - rotX - turnPower) / denominator;
        double backRightPower = (rotY + rotX - turnPower) / denominator;

        hardware.frontLeft.setPower(frontLeftPower);
        hardware.rearLeft.setPower(backLeftPower);
        hardware.frontRight.setPower(frontRightPower);
        hardware.rearRight.setPower(backRightPower);
    }

    public void AutoTurn()
    {
        double error = targetHeading - Math.toDegrees(botHeading);

        if(error > 180)
        {
            error -= 360;
        }
        else if (error < -180) {
            error += 360;
        }

        turnPower = Constants.TURN_P_GAIN * error;
        turnPower = Math.min(Math.max(turnPower, -0.4), 0.4);
    }
}
