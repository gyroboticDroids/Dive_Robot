package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.Objects;

public class Outtake {
    private final Hardware hardware;
    private final Timer actionTimer;

    private boolean driveBack = false;

    private String lastState;

    double vertPosition = 0;

    String state;

    public Outtake(HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);

        actionTimer = new Timer();

        SetState("start");
    }

    private double lastVertConstant = 0;

    public void Update()
    {
        VertSlidesUpdate();

        switch (state)
        {
            case "start":
                hardware.outtakeClaw.setPosition(Constants.OUTTAKE_CLAW_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.outtakeExtension.setPosition(Constants.OUTTAKE_EXTENSION_START);
                    hardware.outtakePivot.setPosition(Constants.OUTTAKE_PIVOT_START);
                    hardware.outtakeWrist.setPosition(Constants.OUTTAKE_WRIST_START);
                }
                break;

            case "transfer intake ready":
                if(!Objects.equals(lastState, "transfer intake ready"))
                {
                    vertPosition = Constants.OUTTAKE_SLIDES_START;
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.outtakeExtension.setPosition(Constants.OUTTAKE_EXTENSION_TRANSFER);

                    hardware.outtakePivot.setPosition(Constants.OUTTAKE_PIVOT_TRANSFER);
                    hardware.outtakeWrist.setPosition(Constants.OUTTAKE_WRIST_TRANSFER);
                }

                if(actionTimer.getElapsedTimeSeconds() > 1)
                {
                    hardware.outtakeClaw.setPosition(Constants.OUTTAKE_CLAW_OPEN);
                }
                else
                {
                    hardware.outtakeClaw.setPosition(Constants.OUTTAKE_CLAW_CLOSED);
                }
                break;

            case "transfer intake":
                hardware.outtakeClaw.setPosition(Constants.OUTTAKE_CLAW_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.outtakeExtension.setPosition(Constants.OUTTAKE_EXTENSION_TRANSFER);

                    hardware.outtakePivot.setPosition(Constants.OUTTAKE_PIVOT_RAISE);
                    hardware.outtakeWrist.setPosition(Constants.OUTTAKE_WRIST_RAISE);
                }
                break;

            case "grab specimen ready":
                if(!Objects.equals(lastState, "grab specimen ready"))
                {
                    vertPosition = Constants.OUTTAKE_SLIDES_SPECIMEN_COLLECT;
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.outtakeExtension.setPosition(Constants.OUTTAKE_EXTENSION_SPECIMEN_COLLECT);
                }

                if(actionTimer.getElapsedTimeSeconds() > 1)
                {
                    hardware.outtakePivot.setPosition(Constants.OUTTAKE_PIVOT_OFF_WALL);
                    hardware.outtakeWrist.setPosition(Constants.OUTTAKE_WRIST_OFF_WALL);
                }

                if(actionTimer.getElapsedTimeSeconds() > 1.5)
                {
                    hardware.outtakeClaw.setPosition(Constants.OUTTAKE_CLAW_OPEN);
                }
                else
                {
                    hardware.outtakeClaw.setPosition(Constants.OUTTAKE_CLAW_CLOSED);
                }
                break;

            case "score specimen ready low":
                if(!Objects.equals(lastState, "score specimen ready low"))
                {
                    lastVertConstant = vertPosition = Constants.OUTTAKE_SLIDES_SPECIMEN_LOW_SCORING;
                }

                hardware.outtakeClaw.setPosition(Constants.OUTTAKE_CLAW_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.outtakeExtension.setPosition(Constants.OUTTAKE_EXTENSION_SPECIMEN_SCORE);
                    hardware.outtakePivot.setPosition(Constants.OUTTAKE_PIVOT_SPECIMEN);
                    hardware.outtakeWrist.setPosition(Constants.OUTTAKE_WRIST_SPECIMEN);
                }
                break;

            case "score specimen ready high":
                if(!Objects.equals(lastState, "score specimen ready high"))
                {
                    lastVertConstant = vertPosition = Constants.OUTTAKE_SLIDES_SPECIMEN_HIGH_SCORING;
                }

                hardware.outtakeClaw.setPosition(Constants.OUTTAKE_CLAW_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.outtakeExtension.setPosition(Constants.OUTTAKE_EXTENSION_SPECIMEN_SCORE);
                    hardware.outtakePivot.setPosition(Constants.OUTTAKE_PIVOT_SPECIMEN);
                    hardware.outtakeWrist.setPosition(Constants.OUTTAKE_WRIST_SPECIMEN);
                }
                break;

            case "score specimen":
                vertPosition = lastVertConstant + Constants.OUTTAKE_SLIDES_SPECIMEN_INCREASE;

                SetState("transfer intake ready");
                break;

            case "score sample ready low":
                if(!Objects.equals(lastState, "score sample ready low"))
                {
                    vertPosition = Constants.OUTTAKE_SLIDES_SAMPLE_LOW;
                }
                break;

            case "score sample ready high":
                if(!Objects.equals(lastState, "score sample ready high"))
                {
                    vertPosition = Constants.OUTTAKE_SLIDES_SAMPLE_HIGH;
                }
                break;

            case "score sample":
                hardware.outtakePivot.setPosition(Constants.OUTTAKE_PIVOT_SAMPLE);
                hardware.outtakeWrist.setPosition(Constants.OUTTAKE_WRIST_SAMPLE);

                hardware.outtakeExtension.setPosition(Constants.OUTTAKE_EXTENSION_SAMPLE_SCORE);

                if(actionTimer.getElapsedTimeSeconds() > 0.75)
                {
                    hardware.outtakeClaw.setPosition(Constants.OUTTAKE_CLAW_OPEN);

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
        vertPosition = MathFunctions.clamp(vertPosition, 0, Constants.OUTTAKE_SLIDES_MAX_LIMIT);

        double error = vertPosition - hardware.outtakeSlide1.getCurrentPosition();

        motorPower = error * Constants.OUTTAKE_SLIDES_P_GAIN;
        motorPower = Math.min(Math.max(motorPower, -0.6), 0.6);

        hardware.outtakeSlide1.setPower(motorPower);
        hardware.outtakeSlide2.setPower(motorPower);
    }

    public boolean IsDriveBack()
    {
        return driveBack;
    }

    public int GetVertSlidePos()
    {
        return hardware.outtakeSlide1.getCurrentPosition();
    }
}
