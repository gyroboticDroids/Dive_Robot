package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;

public class Intake {
    private final Hardware hardware;
    private final Timer actionTimer;

    private boolean onsSetState;
    private boolean onsIntakeState;
    private boolean isBusy = false;
    private boolean intakeWheelsKeepSpinning = false;

    private double timerOffset = 0;

    private double horizontalPosition = 0;
    private int intakeSlideHomeOffset = 0;
    private String state;

    //Color sensor: 0 = none, 1 = yellow, 2 = red, 3 = blue
    private int color = 0;

    public Intake (HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);

        actionTimer = new Timer();

        intakeSlideHomeOffset = -TransferConstants.horiSlidePos;
    }

    public void update() {
        switch (state) {
            case IntakeConstants.START:
                hardware.intakePivot.setPosition(IntakeConstants.PIVOT_START);

                intakeSpeed(IntakeConstants.INTAKE_STOP);

                if (onsSetState)
                {
                    horizontalPosition = IntakeConstants.SLIDES_START;
                }
                if(actionTimer.getElapsedTimeSeconds() > 1 && MathFunctions.roughlyEquals(hardware.intakeSlide.getCurrentPosition() - intakeSlideHomeOffset, horizontalPosition, IntakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case IntakeConstants.RESET_POS:
                if(actionTimer.getElapsedTimeSeconds() > 1)
                {
                    intakeSlideHomeOffset = hardware.intakeSlide.getCurrentPosition();
                    hardware.intakeSlide.setPower(0);
                    isBusy = false;
                }
                else {
                    hardware.intakeSlide.setPower(-0.3);
                }
                break;

            case IntakeConstants.HALFWAY:
                horizontalPosition = IntakeConstants.SLIDES_HALFWAY;
                isBusy = false;
                break;

            case IntakeConstants.INTAKE_SUB_READY:
                if (onsSetState && !(horizontalPosition > IntakeConstants.SLIDES_OUT - 10))
                {
                    horizontalPosition = IntakeConstants.SLIDES_OUT;
                }

                if(horizontalPosition > IntakeConstants.SLIDES_OUT - 10)
                {
                    hardware.intakePivot.setPosition(IntakeConstants.PIVOT_INTERMEDIATE);
                    intakeSpeed(IntakeConstants.INTAKE_FORWARD);
                }

                if(MathFunctions.roughlyEquals(hardware.intakeSlide.getCurrentPosition() - intakeSlideHomeOffset, horizontalPosition, IntakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case IntakeConstants.INTAKE:
                intakeSpeed(IntakeConstants.INTAKE_FORWARD);

                hardware.intakePivot.setPosition(IntakeConstants.PIVOT_DOWN);

                if((actionTimer.getElapsedTimeSeconds() > 0.5))
                {
                    isBusy = false;
                }
                break;

            case IntakeConstants.TRANSFER:
                hardware.intakePivot.setPosition(IntakeConstants.PIVOT_TRANSFER);

                if(getSampleColor() > 0) {
                    if(onsIntakeState){
                        timerOffset = actionTimer.getElapsedTimeSeconds();
                        onsIntakeState = false;
                    }

                    if(actionTimer.getElapsedTimeSeconds() + timerOffset > 0.1) {
                        intakeSpeed(IntakeConstants.INTAKE_STOP);
                    }
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.4)
                {
                    if(!intakeWheelsKeepSpinning) {
                        intakeSpeed(IntakeConstants.INTAKE_STOP);
                    }
                    else if ((hardware.intakeSlide.getCurrentPosition() - intakeSlideHomeOffset) < IntakeConstants.SLIDES_OUT){
                        intakeSpeed(IntakeConstants.INTAKE_STOP);
                    }

                    horizontalPosition = IntakeConstants.SLIDES_TRANSFER;

                    if(MathFunctions.roughlyEquals(hardware.intakeSlide.getCurrentPosition() - intakeSlideHomeOffset, horizontalPosition, IntakeConstants.SLIDES_ACCURACY))
                    {
                        isBusy = false;
                    }
                }
                break;

            case IntakeConstants.REJECT:
                intakeSpeed(IntakeConstants.INTAKE_REVERSE);
                hardware.intakePivot.setPosition(IntakeConstants.PIVOT_INTERMEDIATE);

                if((actionTimer.getElapsedTimeSeconds() > 0.5))
                {
                    isBusy = false;
                }
                break;

            case IntakeConstants.CLEAR_SUB:
                intakeSpeed(IntakeConstants.INTAKE_REVERSE);
                hardware.intakePivot.setPosition(IntakeConstants.PIVOT_DOWN);

                if((actionTimer.getElapsedTimeSeconds() > 0.5))
                {
                    isBusy = false;
                }
                break;
        }
        if(!state.equals(IntakeConstants.RESET_POS)) {
            horizontalSlidesUpdate();
        }
        onsSetState = false;
    }

    public void intakeSpeed(double speed)
    {
        hardware.intakeRight.setPower(speed);
        hardware.intakeLeft.setPower(speed);
    }

    public double getHorizontalPosition() {
        return horizontalPosition;
    }

    public void setHorizontalPosition(double horizontalPosition) {
        this.horizontalPosition = horizontalPosition;
    }

    public String getState() {
        return state;
    }

    public void setState(String s)
    {
        onsSetState = true;
        onsIntakeState = true;
        isBusy = true;
        actionTimer.resetTimer();
        state = s;
        //Update();
    }

    public void horizontalSlidesManual(double position)
    {
        horizontalPosition += position;
        horizontalPosition = MathFunctions.clamp(horizontalPosition, IntakeConstants.SLIDES_OUT, IntakeConstants.SLIDES_MAX);
    }

    public int getSampleColor() {
        if(hardware.colorSensor.red() > 1500 && hardware.colorSensor.green() < 1000) {
            color = 2;//red
        } else if(hardware.colorSensor.red() > 2000 && hardware.colorSensor.green() > 3000) {
            color = 1;//yellow
        } else if(hardware.colorSensor.blue() > 1500) {
            color = 3;//blue
        } else {
            color = 0;
        }
        return color;
    }

    double motorPower;

    public double getMotorPower() {
        return motorPower;
    }

    public void setIntakeWheelsKeepSpinning(boolean val) {
        intakeWheelsKeepSpinning = val;
    }

    public void horizontalSlidesUpdate()
    {
        horizontalPosition = MathFunctions.clamp(horizontalPosition, 0, IntakeConstants.SLIDES_MAX);

        double error = horizontalPosition - (hardware.intakeSlide.getCurrentPosition() - intakeSlideHomeOffset);

        motorPower = error * IntakeConstants.SLIDES_P_GAIN;
        motorPower = Math.min(Math.max(motorPower, -1), 1);

        hardware.intakeSlide.setPower(motorPower);
    }

    public int getHorizontalSlidePos()
    {
        return hardware.intakeSlide.getCurrentPosition() - intakeSlideHomeOffset;
    }

    public boolean isBusy()
    {
        return isBusy;
    }
}
