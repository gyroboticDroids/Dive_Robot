package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

public class Drive {
    private Hardware hardware;

    double y;
    double x;
    double rx;
    boolean resetHeading;

    double botHeading;

    public void Configure(HardwareMap hardwareMap)
    {
        hardware = new Hardware();
        hardware.ConfigureHardware(hardwareMap);
    }

    public void Update(Gamepad gamepad)
    {
        botHeading = hardware.pinpointDriver.getHeading();

        Input(gamepad);
        Movement();
    }

    private void Input(Gamepad gpad)
    {
        double multiplier = (gpad.right_stick_x + gpad.right_stick_y > 0)? Constants.SLOW_SPEED_MULTIPLIER:Constants.DRIVE_SPEED_MULTIPLIER;

        y = -gpad.left_stick_y * multiplier;
        x = gpad.left_stick_x * multiplier;
        rx = (gpad.right_trigger - gpad.left_trigger) * multiplier;
        resetHeading = gpad.back;
    }

    private void Movement()
    {
        if (resetHeading) {
            hardware.pinpointDriver.resetPosAndIMU();
        }

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        hardware.frontLeft.setPower(frontLeftPower);
        hardware.rearLeft.setPower(backLeftPower);
        hardware.frontRight.setPower(frontRightPower);
        hardware.rearRight.setPower(backRightPower);
    }
}
