package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;

public class Drive {
    private final Hardware hardware;
    private final Gamepad gamepad;

    double y;
    double x;
    double rx;

    boolean resetHeading;
    boolean manualTurning = false;

    private double botHeading;
    double targetHeading = 0;

    double turnPower;

    double turnOffset;

    private double drivePower = 0;

    public Drive(HardwareMap hardwareMap, Gamepad gamepad1)
    {
        hardware = new Hardware(hardwareMap);
        hardware.setUpDrive();
        gamepad = gamepad1;

        turnOffset = -Math.toRadians(TransferConstants.heading);
        targetHeading = TransferConstants.heading;
    }

    public void resetHardware() {
        hardware.setToBrake();
    }

    public void update()
    {
        hardware.pinpointDriver.update();

        if(!(drivePower == 0))
        {
            hardware.frontLeft.setPower(drivePower);
            hardware.rearLeft.setPower(drivePower);
            hardware.frontRight.setPower(drivePower);
            hardware.rearRight.setPower(drivePower);

            return;
        }

        botHeading = hardware.pinpointDriver.getHeading() - turnOffset;

        input(gamepad);
        movement();
    }

    private void input(Gamepad gpad)
    {
        double multiplier = (Math.abs(gpad.right_stick_x) + Math.abs(gpad.right_stick_y) > 0)? DriveConstants.DRIVE_SLOW_SPEED_MULTIPLIER : DriveConstants.DRIVE_SPEED_MULTIPLIER;

        y = -gpad.left_stick_y * multiplier;
        x = gpad.left_stick_x * multiplier;
        rx = (gpad.left_trigger - gpad.right_trigger) * 0.4;

        if(Math.abs(gpad.left_trigger + gpad.right_trigger) > 0.05) {
            manualTurning = true;
        } else if (gpad.y || gpad.x || gpad.a || gpad.b) {
            manualTurning = false;
        }

        resetHeading = gpad.back;

        if(manualTurning) {
            targetHeading = Math.toDegrees(botHeading);
            turnPower = rx;
        } else {
            targetHeading = (gpad.y) ? 0 : (gpad.x) ? 90 : (gpad.a) ? -45 : (gpad.b) ? -90 : targetHeading;
        }
    }

    public void autoMovement(double xSpeed, double ySpeed, double targHeading) {
        y = -ySpeed;
        x = xSpeed;
        targetHeading = targHeading;

        hardware.pinpointDriver.update();
        botHeading = hardware.pinpointDriver.getHeading();

        movement();
    }

    private void movement()
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

        if(!manualTurning) {
            autoTurn();
        }

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnPower), 1);
        double frontLeftPower = (rotY + rotX - turnPower) / denominator;
        double backLeftPower = (rotY - rotX - turnPower) / denominator;
        double frontRightPower = (rotY - rotX + turnPower) / denominator;
        double backRightPower = (rotY + rotX + turnPower) / denominator;

        hardware.frontLeft.setPower(frontLeftPower);
        hardware.rearLeft.setPower(backLeftPower);
        hardware.frontRight.setPower(frontRightPower);
        hardware.rearRight.setPower(backRightPower);
    }

    public void autoTurn()
    {
        double error = targetHeading - Math.toDegrees(botHeading);

        if(error > 180)
        {
            error -= 360;
        }
        else if (error < -180) {
            error += 360;
        }

        turnPower = DriveConstants.DRIVE_TURN_P_GAIN * error;
        turnPower = Math.min(Math.max(turnPower, -0.4), 0.4);
    }

    public double getBotHeading() {
        return botHeading;
    }

    public double getTurnPower() {
        return turnPower;
    }

    public boolean isManualTurning() {
        return manualTurning;
    }

    public boolean isDriverInput() {
        return Math.abs(gamepad.left_stick_x) > 0 || Math.abs(gamepad.left_stick_y) > 0;
    }

    public void driveBack(double power) {
        drivePower = power;
    }

    public void updateTurn() {
        targetHeading = Math.toDegrees(hardware.pinpointDriver.getHeading());
    }

    public void resetPowers() {
        hardware.frontLeft.setPower(0);
        hardware.rearLeft.setPower(0);
        hardware.frontRight.setPower(0);
        hardware.rearRight.setPower(0);
    }
}
