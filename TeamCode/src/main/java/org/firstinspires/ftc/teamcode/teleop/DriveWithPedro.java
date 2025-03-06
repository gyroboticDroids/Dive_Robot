package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class DriveWithPedro {
    private final Gamepad gamepad;
    private final Follower follower;

    double y;
    double x;
    double rx;

    boolean manualTurning = false;
    double targetHeading = 0;

    boolean ons = false;

    double turnOffset;

    private double drivePower = 0;

    public DriveWithPedro(HardwareMap hardwareMap, Gamepad gamepad1)
    {
        gamepad = gamepad1;

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(TransferConstants.endPose);
    }

    public void update()
    {
        if(!(drivePower == 0))
        {
            follower.setTeleOpMovementVectors(0.5, 0, 0, true);
        }
        else {
            movement();
        }

        follower.update();
    }

    private void movement()
    {
        if (gamepad.back)
        {
            follower.setHeadingOffset(0);
            targetHeading = 0;
        }

        double multiplier = (Math.abs(gamepad.right_stick_x) + Math.abs(gamepad.right_stick_y) > 0)? DriveConstants.DRIVE_SLOW_SPEED_MULTIPLIER : DriveConstants.DRIVE_SPEED_MULTIPLIER;

        y = -gamepad.left_stick_y * multiplier;
        x = gamepad.left_stick_x * multiplier;
        rx = (gamepad.left_trigger - gamepad.right_trigger) * 0.4;
        manualTurning = Math.abs(gamepad.left_trigger + gamepad.right_trigger) > 0.05;

        follower.setTeleOpMovementVectors(y, x, rx, false);

        if (!manualTurning) {
            autoTurn();
        }else {
            ons = true;
        }
    }

    public void autoTurn()
    {
        if(ons) {
            updateTurn();
            ons = false;
        }

        targetHeading = (gamepad.y) ? 0 : (gamepad.x) ? 90 : (gamepad.a) ? -45 : (gamepad.b) ? -90 : targetHeading;

        //follower.turnTo(Math.toRadians(targetHeading));
    }

    public double getBotHeading() {
        return follower.getPose().getHeading();
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
        targetHeading = Math.toDegrees(follower.getPose().getHeading());
    }

    public void resetPowers() {
        follower.setTeleOpMovementVectors(0,0,0);
    }
}
