package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Intake {

    private final Hardware hardware;
    private final Timer actionTimer;

    private boolean onsSetState;
    private boolean isBusy = false;

    public double horizontalPosition = 0;

    String state;

    public Intake (HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);

        actionTimer = new Timer();

        SetState("start");
    }

    public void Update() {
        HorizontalSlidesUpdate();

        switch (state) {
            case "start":
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

            case "intake sub ready":
                if (onsSetState)
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

            case "intake":
                IntakeSpeed(IntakeConstants.INTAKE_FORWARD);

                hardware.intakePivot.setPosition(IntakeConstants.PIVOT_DOWN);

                horizontalPosition = MathFunctions.clamp(horizontalPosition, IntakeConstants.SLIDES_OUT, IntakeConstants.SLIDES_MAX);

                if((actionTimer.getElapsedTimeSeconds() > 0.5))
                {
                    isBusy = false;
                }
                break;

            case "transfer":
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

            case "reject":
                IntakeSpeed(IntakeConstants.INTAKE_REVERSE);
                hardware.intakePivot.setPosition(IntakeConstants.PIVOT_INTERMEDIATE);

                horizontalPosition = MathFunctions.clamp(horizontalPosition, IntakeConstants.SLIDES_OUT, IntakeConstants.SLIDES_MAX);

                if((actionTimer.getElapsedTimeSeconds() > 0.5))
                {
                    isBusy = false;
                }
                break;
        }

        onsSetState = false;
    }

    public void IntakeSpeed(double speed)
    {
        hardware.intakeRight.setPower(speed);
        hardware.intakeLeft.setPower(speed);
    }

    public void SetState(String s)
    {
        onsSetState = true;
        isBusy = true;
        actionTimer.resetTimer();
        state = s;
        Update();
    }

    public void HorizontalSlidesManual(double position)
    {
        horizontalPosition += position;
    }

    public void HorizontalSlidesUpdate()
    {
        horizontalPosition = MathFunctions.clamp(horizontalPosition, 0, IntakeConstants.SLIDES_MAX);

        double error = horizontalPosition - hardware.intakeSlide.getCurrentPosition();

        double motorPower = error * IntakeConstants.SLIDES_P_GAIN;
        motorPower = Math.min(Math.max(motorPower, -0.6), 0.6);

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
