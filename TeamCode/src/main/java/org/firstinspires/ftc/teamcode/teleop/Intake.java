package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Intake {
    private final Hardware hardware;
    private final Timer actionTimer;

    private boolean onsSetState;
    private boolean isBusy = false;

    private double horizontalPosition = 0;
    private String state;

    public Intake (HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);

        actionTimer = new Timer();

        setState(IntakeConstants.START);
    }

    public void Update() {
        switch (state) {
            case IntakeConstants.START:
                hardware.intakePivot.setPosition(IntakeConstants.PIVOT_START);

                IntakeSpeed(IntakeConstants.INTAKE_STOP);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    if (onsSetState)
                    {
                        horizontalPosition = IntakeConstants.SLIDES_START;
                    }

                    if(MathFunctions.roughlyEquals(hardware.intakeSlide.getCurrentPosition(), horizontalPosition, IntakeConstants.SLIDES_ACCURACY))
                    {
                        isBusy = false;
                    }
                }
                break;

            case IntakeConstants.RESET_POS:
                hardware.intakeSlide.setPower(-0.4);

                if(actionTimer.getElapsedTimeSeconds() > 1)
                {
                    hardware.intakeSlide.setPower(0);
                }
                if(actionTimer.getElapsedTimeSeconds() > 1.5)
                {
                    hardware.intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    hardware.intakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                    IntakeSpeed(IntakeConstants.INTAKE_FORWARD);

                    horizontalPosition = MathFunctions.clamp(horizontalPosition, IntakeConstants.SLIDES_OUT, IntakeConstants.SLIDES_MAX);
                }

                if(MathFunctions.roughlyEquals(hardware.intakeSlide.getCurrentPosition(), horizontalPosition, IntakeConstants.SLIDES_ACCURACY))
                {
                    isBusy = false;
                }
                break;

            case IntakeConstants.INTAKE:
                IntakeSpeed(IntakeConstants.INTAKE_FORWARD);

                hardware.intakePivot.setPosition(IntakeConstants.PIVOT_DOWN);

                horizontalPosition = MathFunctions.clamp(horizontalPosition, IntakeConstants.SLIDES_OUT, IntakeConstants.SLIDES_MAX);

                if((actionTimer.getElapsedTimeSeconds() > 0.5))
                {
                    isBusy = false;
                }
                break;

            case IntakeConstants.TRANSFER:
                IntakeSpeed(IntakeConstants.INTAKE_STOP);

                hardware.intakePivot.setPosition(IntakeConstants.PIVOT_TRANSFER);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    horizontalPosition = IntakeConstants.SLIDES_TRANSFER;

                    if(MathFunctions.roughlyEquals(hardware.intakeSlide.getCurrentPosition(), horizontalPosition, IntakeConstants.SLIDES_ACCURACY))
                    {
                        isBusy = false;
                    }
                }
                break;

            case IntakeConstants.REJECT:
                IntakeSpeed(IntakeConstants.INTAKE_REVERSE);
                hardware.intakePivot.setPosition(IntakeConstants.PIVOT_INTERMEDIATE);

                horizontalPosition = MathFunctions.clamp(horizontalPosition, IntakeConstants.SLIDES_OUT, IntakeConstants.SLIDES_MAX);

                if((actionTimer.getElapsedTimeSeconds() > 0.5))
                {
                    isBusy = false;
                }
                break;

            case IntakeConstants.CLEAR_SUB:
                IntakeSpeed(IntakeConstants.INTAKE_REVERSE);
                hardware.intakePivot.setPosition(IntakeConstants.PIVOT_DOWN);

                horizontalPosition = MathFunctions.clamp(horizontalPosition, IntakeConstants.SLIDES_OUT, IntakeConstants.SLIDES_MAX);

                if((actionTimer.getElapsedTimeSeconds() > 0.5))
                {
                    isBusy = false;
                }
                break;
        }
        if(!state.equals(IntakeConstants.RESET_POS)) {
            HorizontalSlidesUpdate();
        }
        onsSetState = false;
    }

    public void IntakeSpeed(double speed)
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

    public void HorizontalSlidesManual(double position)
    {
        horizontalPosition += position;
    }

    double motorPower;

    public double getMotorPower() {
        return motorPower;
    }

    public void HorizontalSlidesUpdate()
    {
        horizontalPosition = MathFunctions.clamp(horizontalPosition, 0, IntakeConstants.SLIDES_MAX);

        double error = horizontalPosition - hardware.intakeSlide.getCurrentPosition();

        motorPower = error * IntakeConstants.SLIDES_P_GAIN;
        motorPower = Math.min(Math.max(motorPower, -1), 1);

        hardware.intakeSlide.setPower(motorPower);
    }

    public int GetHorizontalSlidePos()
    {
        return hardware.intakeSlide.getCurrentPosition();
    }

    public boolean IsBusy()
    {
        return isBusy;
    }
}
