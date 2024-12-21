package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class VerticalExtension {
    private final Hardware hardware;
    private final Outtake outtake;

    private boolean isActive;
    private boolean driveBack = false;

    private final Timer actionTimer;

    double vertPosition = 0;

    String state;

    public VerticalExtension (HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);
        outtake = new Outtake(hardwareMap);

        actionTimer = new Timer();

        SetState("start");
    }

    double lastVertConstant = 0;

    public void Update()
    {
        VertSlidesUpdate();

        isActive = hardware.vertSlide1.isBusy();

        switch (state)
        {
            case "start":
                outtake.SetState("start");

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.specimenExtension.setPosition(Constants.EXTENSION_BACK);
                }
                break;

            case "transfer intake ready":

                vertPosition = Constants.VERT_SLIDES_TRANSFER;
                outtake.SetState("transfer intake ready");

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.specimenExtension.setPosition(Constants.EXTENSION_TRANSFER);
                }
                break;

            case "transfer intake":
                outtake.SetState("transfer intake");

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.specimenExtension.setPosition(Constants.EXTENSION_TRANSFER);
                }
                break;

            case "grab specimen ready":
                vertPosition = Constants.VERT_SLIDES_SPECIMEN_COLLECT;
                outtake.SetState("grab specimen ready");

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.specimenExtension.setPosition(Constants.EXTENSION_SPECIMEN_COLLECT);
                }
                break;

            case "score specimen ready low":
                lastVertConstant = vertPosition = Constants.VERT_SLIDES_SPECIMEN_LOW_SCORING;

                outtake.SetState("score specimen ready low");

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.specimenExtension.setPosition(Constants.EXTENSION_SPECIMEN_SCORE);
                }
                break;

            case "score specimen ready high":
                lastVertConstant = vertPosition = Constants.VERT_SLIDES_SPECIMEN_HIGH_SCORING;

                outtake.SetState("score specimen ready");

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.specimenExtension.setPosition(Constants.EXTENSION_SPECIMEN_SCORE);
                }
                break;

            case "score specimen":
                vertPosition = lastVertConstant + Constants.VERT_SLIDES_SPECIMEN_INCREASE;

                SetState("transfer intake ready");
                break;

            case "score sample ready low":
                vertPosition = Constants.VERT_SLIDES_SAMPLE_LOW;
                break;

            case "score sample ready high":
                vertPosition = Constants.VERT_SLIDES_SAMPLE_HIGH;
                break;

            case "score sample":
                outtake.SetState("score sample");

                hardware.specimenExtension.setPosition(Constants.EXTENSION_SAMPLE_SCORE);

                if(actionTimer.getElapsedTimeSeconds() > 0.75)
                {
                    driveBack = true;
                }

                if(actionTimer.getElapsedTimeSeconds() > 1.5)
                {
                    driveBack = false;
                    SetState("transfer intake ready");
                }
                break;
        }

    }
    public void SetState(String s)
    {
        actionTimer.resetTimer();
        state = s;
        Update();
    }

    public void VertSlidesUpdate()
    {
        double error = vertPosition - hardware.vertSlide1.getCurrentPosition();

        double motorPower = error * Constants.VERT_SLIDES_P_GAIN;
        motorPower = Math.min(Math.max(motorPower, -0.6), 0.6);

        hardware.vertSlide1.setPower(motorPower);
        hardware.vertSlide2.setPower(motorPower);
    }

    public boolean IsActive()
    {
        return isActive;
    }

    public boolean IsDriveBack()
    {
        return driveBack;
    }
}
