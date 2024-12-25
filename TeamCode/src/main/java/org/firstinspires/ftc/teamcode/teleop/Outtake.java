package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Outtake {
    private final Hardware hardware;
    private final Timer actionTimer;

    private boolean driveBack = false;

    private String lastState;

    private double vertPosition = 0;

    String state;

    public Outtake(HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);

        actionTimer = new Timer();

        SetState("start");
    }

    double lastVertConstant = 0;

    public void Update()
    {
        VertSlidesUpdate();

        switch (state)
        {
            case "start":
                hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.specimenExtension.setPosition(Constants.EXTENSION_START);
                    hardware.pivot.setPosition(Constants.PIVOT_START);
                    hardware.wrist.setPosition(Constants.WRIST_START);
                }
                break;

            case "transfer intake ready":
                if(!(lastState == "transfer intake ready"))
                {
                    vertPosition = Constants.VERT_SLIDES_START;
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.specimenExtension.setPosition(Constants.EXTENSION_TRANSFER);

                    hardware.pivot.setPosition(Constants.PIVOT_TRANSFER);
                    hardware.wrist.setPosition(Constants.WRIST_TRANSFER);
                }

                if(actionTimer.getElapsedTimeSeconds() > 1)
                {
                    hardware.outtake.setPosition(Constants.OUTTAKE_OPEN);
                }
                else
                {
                    hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);
                }
                break;

            case "transfer intake":
                hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.specimenExtension.setPosition(Constants.EXTENSION_TRANSFER);

                    hardware.pivot.setPosition(Constants.PIVOT_RAISE);
                    hardware.wrist.setPosition(Constants.WRIST_RAISE);
                }
                break;

            case "grab specimen ready":
                if(!(lastState == "grab specimen ready"))
                {
                    vertPosition = Constants.VERT_SLIDES_SPECIMEN_COLLECT;
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.specimenExtension.setPosition(Constants.EXTENSION_SPECIMEN_COLLECT);

                    hardware.pivot.setPosition(Constants.PIVOT_OFF_WALL);
                    hardware.wrist.setPosition(Constants.WRIST_OFF_WALL);
                }

                if(actionTimer.getElapsedTimeSeconds() > 1)
                {
                    hardware.outtake.setPosition(Constants.OUTTAKE_OPEN);
                }
                else
                {
                    hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);
                }
                break;

            case "score specimen ready low":
                if(!(lastState == "score specimen ready low"))
                {
                    lastVertConstant = vertPosition = Constants.VERT_SLIDES_SPECIMEN_LOW_SCORING;
                }

                hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.specimenExtension.setPosition(Constants.EXTENSION_SPECIMEN_SCORE);
                    hardware.pivot.setPosition(Constants.PIVOT_SPECIMEN);
                    hardware.wrist.setPosition(Constants.WRIST_SPECIMEN);
                }
                break;

            case "score specimen ready high":
                if(!(lastState == "score specimen ready high"))
                {
                    lastVertConstant = vertPosition = Constants.VERT_SLIDES_SPECIMEN_HIGH_SCORING;
                }

                hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.specimenExtension.setPosition(Constants.EXTENSION_SPECIMEN_SCORE);
                    hardware.pivot.setPosition(Constants.PIVOT_SPECIMEN);
                    hardware.wrist.setPosition(Constants.WRIST_SPECIMEN);
                }
                break;

            case "score specimen":
                vertPosition = lastVertConstant + Constants.VERT_SLIDES_SPECIMEN_INCREASE;

                SetState("transfer intake ready");
                break;

            case "score sample ready low":
                if(!(lastState == "score sample ready low"))
                {
                    vertPosition = Constants.VERT_SLIDES_SAMPLE_LOW;
                }
                break;

            case "score sample ready high":
                if(!(lastState == "score sample ready high"))
                {
                    vertPosition = Constants.VERT_SLIDES_SAMPLE_HIGH;
                }
                break;

            case "score sample":
                hardware.pivot.setPosition(Constants.PIVOT_SAMPLE);
                hardware.wrist.setPosition(Constants.WRIST_SAMPLE);

                hardware.specimenExtension.setPosition(Constants.EXTENSION_SAMPLE_SCORE);

                if(actionTimer.getElapsedTimeSeconds() > 0.75)
                {
                    hardware.pivot.setPosition(Constants.OUTTAKE_OPEN);

                    driveBack = true;
                }

                if(actionTimer.getElapsedTimeSeconds() > 1.5)
                {
                    driveBack = false;
                    SetState("transfer intake ready");
                }
                break;
        }

        lastState = state;
    }
    public void SetState(String s)
    {
        actionTimer.resetTimer();
        state = s;
        Update();
    }

    public void VertSlidesManual(double position)
    {
        vertPosition += position * 10;
    }

    double motorPower;

    public void VertSlidesUpdate()
    {
        vertPosition = MathFunctions.clamp(vertPosition, 0, Constants.VERT_SLIDES_MAX_LIMIT);

        double error = vertPosition - hardware.vertSlide1.getCurrentPosition();

        motorPower = error * Constants.VERT_SLIDES_P_GAIN;
        motorPower = Math.min(Math.max(motorPower, -0.6), 0.6);

        hardware.vertSlide1.setPower(motorPower);
        hardware.vertSlide2.setPower(motorPower);
    }

    public boolean IsDriveBack()
    {
        return driveBack;
    }

    public int GetVertSlidePos()
    {
        return hardware.vertSlide1.getCurrentPosition();
    }
}
