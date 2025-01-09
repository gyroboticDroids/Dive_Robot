package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

public class Intake {
    private final Hardware hardware;
    private final Timer actionTimer;

    private boolean onsSetState;
    private boolean isBusy = false;

    private double horizontalPosition = 0;
    private int intakeSlideHomeOffset = 0;
    private String state;

    public Intake (HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);

        actionTimer = new Timer();

        setState(IntakeConstants.START);
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
                hardware.intakeSlide.setPower(-0.3);

                if(actionTimer.getElapsedTimeSeconds() > 1)
                {
                    intakeSlideHomeOffset = hardware.intakeSlide.getCurrentPosition();
                    hardware.intakeSlide.setPower(0);
                    isBusy = false;
                }
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

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    intakeSpeed(IntakeConstants.INTAKE_STOP);
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

    double motorPower;

    public double getMotorPower() {
        return motorPower;
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
