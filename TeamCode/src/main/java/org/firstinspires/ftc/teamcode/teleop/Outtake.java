package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Outtake {
    private final Hardware hardware;
    private final Timer actionTimer;

    private boolean driveBack = false;
    private boolean isBusy = false;
    private boolean onsSetState = false;
    private boolean specimenOnsSetState = false;
    private boolean ons = false;

    private double vertPosition = 0;

    private String state;

    public Outtake(HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);

        actionTimer = new Timer();

        setState(OuttakeConstants.START);
    }

    private double lastVertConstant = 0;

    public void Update()
    {
        switch (state)
        {
            case OuttakeConstants.START:
                if(onsSetState)
                {
                    vertPosition = OuttakeConstants.SLIDES_START;
                }

                hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_START);
                    hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_START);
                    hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_START);
                }

                if(actionTimer.getElapsedTimeSeconds() > 1 && MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.TRANSFER_INTAKE_READY:
                if(onsSetState)
                {
                    vertPosition = OuttakeConstants.SLIDES_START;
                }

                hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_TRANSFER);
                hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_TRANSFER);
                hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_TRANSFER);

                hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_OPEN);

                if(actionTimer.getElapsedTimeSeconds() > 1 && MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.TRANSFER_INTAKE:
                hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_TRANSFER);
                    hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_RAISE);
                    hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_RAISE);
                }

                if(actionTimer.getElapsedTimeSeconds() > 1)
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.GRAB_SPECIMEN_READY:
                if(onsSetState)
                {
                    vertPosition = OuttakeConstants.SLIDES_SPECIMEN_COLLECT;
                }

                hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_SPECIMEN_OFF_WALL);

                if(actionTimer.getElapsedTimeSeconds() > 0.25)
                {
                    hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_OFF_WALL);
                    hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_OFF_WALL);
                }

                if(actionTimer.getElapsedTimeSeconds() > 1.5)
                {
                    hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_OPEN);
                }

                if(actionTimer.getElapsedTimeSeconds() > 1.75 && MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.SCORE_SPECIMEN_READY_LOW:
                if(specimenOnsSetState && actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    lastVertConstant = vertPosition = OuttakeConstants.SLIDES_SPECIMEN_LOW_SCORING;
                    specimenOnsSetState = false;
                }

                hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_SPECIMEN);
                    hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_SPECIMEN);

                    driveBack = true;
                }

                if(actionTimer.getElapsedTimeSeconds() > 1)
                {
                    hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_SPECIMEN_SCORE);

                    driveBack = false;
                }

                if(actionTimer.getElapsedTimeSeconds() > 1.5 && MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.SCORE_SPECIMEN_READY_HIGH:
                if(specimenOnsSetState && actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    lastVertConstant = vertPosition = OuttakeConstants.SLIDES_SPECIMEN_HIGH_SCORING;
                    specimenOnsSetState = false;
                }

                hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_SPECIMEN);
                    hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_SPECIMEN);

                    driveBack = true;
                }

                if(actionTimer.getElapsedTimeSeconds() > 1)
                {
                    hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_SPECIMEN_SCORE);

                    driveBack = false;
                }

                if(actionTimer.getElapsedTimeSeconds() > 1.5 && MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.SCORE_SPECIMEN:
                if(onsSetState)
                {
                    vertPosition = lastVertConstant + OuttakeConstants.SLIDES_SPECIMEN_INCREASE;
                    ons = true;
                }

                if(MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    if(ons)
                    {
                        actionTimer.resetTimer();
                        ons = false;
                    }
                    hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_OPEN);

                    if(actionTimer.getElapsedTimeSeconds() > 0.5)
                    {
                        isBusy = false;
                    }
                }
                break;

            case OuttakeConstants.SCORE_SAMPLE_READY_LOW:
                hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_SAMPLE);
                hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_SAMPLE);
                hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_SAMPLE_SCORE);

                if(onsSetState)
                {
                    vertPosition = OuttakeConstants.SLIDES_SAMPLE_LOW;
                }

                if(MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.SCORE_SAMPLE_READY_HIGH:
                hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_SAMPLE);
                hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_SAMPLE);
                hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_SAMPLE_SCORE);

                if(onsSetState)
                {
                    vertPosition = OuttakeConstants.SLIDES_SAMPLE_HIGH;
                }

                if(MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.SCORE_SAMPLE:
                hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_OPEN);

                driveBack = true;

                if(actionTimer.getElapsedTimeSeconds() > 0.75)
                {
                    driveBack = false;
                    isBusy = false;
                }
                break;
        }
        VertSlidesUpdate();
        onsSetState = false;
    }


    public String getState() {
        return state;
    }

    public void setState(String s)
    {
        isBusy = true;
        onsSetState = true;
        specimenOnsSetState = true;
        actionTimer.resetTimer();
        state = s;
        //Update();
    }

    public double getVertPosition() {
        return vertPosition;
    }

    public void setVertPosition(double vertPosition) {
        this.vertPosition = vertPosition;
    }

    public void VertSlidesManual(double position)
    {
        vertPosition += position * 10;
    }

    double motorPower;

    public void VertSlidesUpdate()
    {
        vertPosition = MathFunctions.clamp(vertPosition, 0, OuttakeConstants.SLIDES_MAX_LIMIT);

        double error = vertPosition - hardware.outtakeSlide1.getCurrentPosition();

        motorPower = error * OuttakeConstants.SLIDES_P_GAIN;

        if(hardware.outtakeSlide1.getCurrentPosition() < OuttakeConstants.SLIDES_ACCURACY && vertPosition == 0) {
            hardware.outtakeSlide1.setPower(0);
            hardware.outtakeSlide2.setPower(0);

            hardware.outtakeSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hardware.outtakeSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else {
            motorPower = Math.min(Math.max(motorPower, -0.8), 0.8);

            hardware.outtakeSlide1.setPower(motorPower);
            hardware.outtakeSlide2.setPower(motorPower);
        }
    }

    public boolean IsDriveBack()
    {
        return driveBack;
    }

    public boolean IsBusy()
    {
        return isBusy;
    }

    public int GetVertSlidePos()
    {
        return hardware.outtakeSlide1.getCurrentPosition();
    }
}