package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.Objects;

public class Intake {

    private final Hardware hardware;
    private final Timer actionTimer;

    private String lastState;

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
                hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_START);

                IntakeSpeed(Constants.INTAKE_STOP);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    horizontalPosition = Constants.INTAKE_SLIDES_START;
                }
                break;

            case "intake sub ready":
                if (!Objects.equals(lastState, "intake sub ready"))
                {
                    horizontalPosition = Constants.INTAKE_SLIDES_OUT;
                }

                if(horizontalPosition > Constants.INTAKE_SLIDES_OUT - 10)
                {
                    hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_INTERMEDIATE);
                    IntakeSpeed(Constants.INTAKE_FORWARD);

                    horizontalPosition = MathFunctions.clamp(horizontalPosition, Constants.INTAKE_SLIDES_OUT, Constants.INTAKE_SLIDES_MAX);
                }
                break;

            case "intake":
                IntakeSpeed(Constants.INTAKE_FORWARD);

                hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_DOWN);

                horizontalPosition = MathFunctions.clamp(horizontalPosition, Constants.INTAKE_SLIDES_OUT, Constants.INTAKE_SLIDES_MAX);
                break;

            case "transfer":
                IntakeSpeed(Constants.INTAKE_STOP);

                hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_TRANSFER);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    horizontalPosition = Constants.INTAKE_SLIDES_TRANSFER;
                }
                break;

            case "reject":
                IntakeSpeed(Constants.INTAKE_REVERSE);
                hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_INTERMEDIATE);

                horizontalPosition = MathFunctions.clamp(horizontalPosition, Constants.INTAKE_SLIDES_OUT, Constants.INTAKE_SLIDES_MAX);
                break;
        }

        lastState = state;
    }

    public void IntakeSpeed(double speed)
    {
        hardware.intakeRight.setPower(speed);
        hardware.intakeLeft.setPower(speed);
    }

    public void SetState(String s)
    {
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
        horizontalPosition = MathFunctions.clamp(horizontalPosition, 0, Constants.INTAKE_SLIDES_MAX);

        double error = horizontalPosition - hardware.intakeSlide.getCurrentPosition();

        double motorPower = error * Constants.INTAKE_SLIDES_P_GAIN;
        motorPower = Math.min(Math.max(motorPower, -0.6), 0.6);

        hardware.intakeSlide.setPower(motorPower);
    }

    public int GetHorizontalSlidePos()
    {
        return hardware.intakeSlide.getCurrentPosition();
    }
}
