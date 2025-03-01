package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;

public class Outtake {
    private final Hardware hardware;
    private final Timer actionTimer;

    private boolean driveBack = false;
    private boolean isBusy = false;
    private boolean onsSetState = false;
    private boolean specimenOnsSetState = false;
    private boolean vertOneShot = true;

    private boolean fromTransfer = true;

    private boolean hanging = false;

    private double vertPosition = 0;

    private String state;

    public Outtake(HardwareMap hardwareMap) {
        hardware = new Hardware(hardwareMap);

        actionTimer = new Timer();

        setState(OuttakeConstants.START);
    }

    public void update() {
        switch (state)
        {
            case OuttakeConstants.START:
                if(onsSetState)
                {
                    if(!hanging) {
                        vertPosition = OuttakeConstants.SLIDES_START;
                    } else {
                        vertPosition = OuttakeConstants.SLIDES_HANG;
                    }
                }

                hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.25)
                {
                    hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_START);
                    hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_START);
                    hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_START);
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.5 && MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case IntakeConstants.RESET_POS:
                if(actionTimer.getElapsedTimeSeconds() > 1)
                {
                    hardware.outtakeSlide1.setPower(0);
                    hardware.outtakeSlide2.setPower(0);

                    hardware.outtakeSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    hardware.outtakeSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    isBusy = false;
                }
                else {
                    hardware.outtakeSlide1.setPower(-0.3);
                    hardware.outtakeSlide2.setPower(-0.3);
                }
                break;

            case OuttakeConstants.TRANSFER_INTAKE_READY:
                if(onsSetState)
                {
                    vertPosition = OuttakeConstants.SLIDES_START;
                }

                hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_TRANSFER);
                hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_TRANSFER_READY);
                hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_TRANSFER_READY);

                hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_OPEN);

                if(actionTimer.getElapsedTimeSeconds() > 0.3)
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.TRANSFER_INTAKE:
                hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_TRANSFER);
                hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_TRANSFER);
                hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_TRANSFER);

                if(actionTimer.getElapsedTimeSeconds() > 0.1)
                {
                    hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_CLOSED);
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.3)
                {
                    vertPosition = OuttakeConstants.SLIDES_TRANSFER_UP;
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.35 && MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.GRAB_SPECIMEN_READY:
                if(onsSetState){
                    fromTransfer = vertPosition == OuttakeConstants.SLIDES_TRANSFER_UP;
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.2 && specimenOnsSetState) {
                    vertPosition = OuttakeConstants.SLIDES_SPECIMEN_COLLECT;
                    specimenOnsSetState = false;
                }

                if(!fromTransfer) {
                    if(actionTimer.getElapsedTimeSeconds() > 0.1) {
                        hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_SPECIMEN_OFF_WALL);
                    }

                    if(actionTimer.getElapsedTimeSeconds() > 0.7) {
                        hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_OFF_WALL);
                        hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_OFF_WALL);
                    }
                }
                else {
                    hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_SPECIMEN_OFF_WALL);
                    hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_OFF_WALL);
                    hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_OFF_WALL);
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.75)
                {
                    hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_OPEN);
                }
                else {
                    hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_CLOSED);
                }

                if(actionTimer.getElapsedTimeSeconds() > 1 && MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.SCORE_SPECIMEN_READY_LOW:
                if(specimenOnsSetState && actionTimer.getElapsedTimeSeconds() > 0.25)
                {
                    vertPosition = OuttakeConstants.SLIDES_SPECIMEN_LOW_SCORING;
                    specimenOnsSetState = false;
                }

                hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.25)
                {
                    hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_SPECIMEN);
                    hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_SPECIMEN);

                    //driveBack = true;
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_SPECIMEN_SCORE);

                    //driveBack = false;
                }

                if(actionTimer.getElapsedTimeSeconds() > 1 && MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.SCORE_SPECIMEN_READY_HIGH:
                if(specimenOnsSetState && actionTimer.getElapsedTimeSeconds() > 0.25)
                {
                    vertPosition = OuttakeConstants.SLIDES_SPECIMEN_HIGH_SCORING;
                    specimenOnsSetState = false;
                }

                hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.25)
                {
                    hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_SPECIMEN);
                    hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_SPECIMEN);

                    //driveBack = true;
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_SPECIMEN_SCORE);

                    //driveBack = false;
                }

                if(actionTimer.getElapsedTimeSeconds() > 1 && MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.SCORE_SPECIMEN:
                hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_RAISE);
                hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_RAISE);


                if(actionTimer.getElapsedTimeSeconds() > 0.2)
                {
                    hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_OPEN);
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.25)
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.SCORE_SAMPLE_READY_LOW:
                if(onsSetState)
                {
                    vertPosition = OuttakeConstants.SLIDES_SAMPLE_LOW;
                }

                if(MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_PIVOT_CLEAR))
                {
                    hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_SAMPLE);
                    hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_SAMPLE);
                    hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_SAMPLE_SCORE);
                }
                else {
                    hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_RAISE);
                    hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_RAISE);
                    hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_TRANSFER);
                }

                if(MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.SCORE_SAMPLE_READY_HIGH:
                if(onsSetState)
                {
                    vertPosition = OuttakeConstants.SLIDES_SAMPLE_HIGH;
                }

                if(MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_PIVOT_CLEAR))
                {
                    hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_SAMPLE);
                    hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_SAMPLE);
                    hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_SAMPLE_SCORE);
                }
                else {
                    hardware.outtakePivot.setPosition(OuttakeConstants.PIVOT_RAISE);
                    hardware.outtakeWrist.setPosition(OuttakeConstants.WRIST_RAISE);
                    hardware.outtakeExtension.setPosition(OuttakeConstants.EXTENSION_TRANSFER);
                }

                if(MathFunctions.roughlyEquals(hardware.outtakeSlide1.getCurrentPosition(), vertPosition, OuttakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case OuttakeConstants.SCORE_SAMPLE:
                hardware.outtakeClaw.setPosition(OuttakeConstants.CLAW_OPEN);

                driveBack = true;

                if(actionTimer.getElapsedTimeSeconds() > 0.25)
                {
                    driveBack = false;
                    isBusy = false;
                }
                break;
        }
        if(!state.equals(OuttakeConstants.RESET_POS)) {
            vertSlidesUpdate();
        }
        onsSetState = false;
    }


    public String getState() {
        return state;
    }

    public void setState(String s) {
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

    public void vertSlidesManual(double position)
    {
        vertPosition += position * 10;
    }

    double motorPower;

    public void vertSlidesUpdate() {
        vertPosition = MathFunctions.clamp(vertPosition, 0, OuttakeConstants.SLIDES_MAX_LIMIT);

        double error = vertPosition - hardware.outtakeSlide1.getCurrentPosition();

        motorPower = error * ((!hanging)? OuttakeConstants.SLIDES_P_GAIN:OuttakeConstants.SLIDES_HANGING_P_GAIN);

        if(hardware.outtakeSlide1.getCurrentPosition() < OuttakeConstants.SLIDES_ACCURACY && vertPosition == 0) {
            if(vertOneShot) {
                hardware.outtakeSlide1.setPower(0);
                hardware.outtakeSlide2.setPower(0);

                //hardware.outtakeSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //hardware.outtakeSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vertOneShot = false;
            }
        }
        else {
            motorPower = Math.min(Math.max(motorPower, -1), 1);

            hardware.outtakeSlide1.setPower(motorPower);
            hardware.outtakeSlide2.setPower(motorPower);

            vertOneShot = true;
        }
    }

    public boolean isDriveBack()
    {
        return driveBack;
    }

    public void setHanging(boolean hanging) {
        this.hanging = hanging;
    }

    public boolean isBusy()
    {
        return isBusy;
    }

    public int getVertSlidePos()
    {
        return hardware.outtakeSlide1.getCurrentPosition();
    }
}