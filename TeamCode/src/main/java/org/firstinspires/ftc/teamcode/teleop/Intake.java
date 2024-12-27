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
                    horizontalPosition = Constants.HORIZONTAL_SLIDES_START;
                }
                break;

            case "intake sub ready":
                if (!Objects.equals(lastState, "intake sub ready"))
                {
                    horizontalPosition = Constants.HORIZONTAL_SLIDES_OUT;
                }

                if(horizontalPosition > Constants.HORIZONTAL_SLIDES_OUT - 10)
                {
                    hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_INTERMEDIATE);
                    IntakeSpeed(Constants.INTAKE_FORWARD);

                    horizontalPosition = MathFunctions.clamp(horizontalPosition, Constants.HORIZONTAL_SLIDES_OUT, Constants.HORIZONTAL_SLIDES_MAX);
                }
                break;

            case "intake":
                IntakeSpeed(Constants.INTAKE_FORWARD);

                hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_DOWN);

                horizontalPosition = MathFunctions.clamp(horizontalPosition, Constants.HORIZONTAL_SLIDES_OUT, Constants.HORIZONTAL_SLIDES_MAX);
                break;

            case "transfer":
                IntakeSpeed(Constants.INTAKE_STOP);

                hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_TRANSFER);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    horizontalPosition = Constants.HORIZONTAL_SLIDES_TRANSFER;
                }
                break;

            case "reject":
                IntakeSpeed(Constants.INTAKE_REVERSE);
                hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_INTERMEDIATE);

                horizontalPosition = MathFunctions.clamp(horizontalPosition, Constants.HORIZONTAL_SLIDES_OUT, Constants.HORIZONTAL_SLIDES_MAX);
                break;
        }

        lastState = state;
    }

    public void IntakeSpeed(double speed)
    {
        hardware.intake1.setPower(speed);
        hardware.intake2.setPower(speed);
    }

    public void SetState(String s)
    {
        actionTimer.resetTimer();
        state = s;
        Update();
    }

    public void HorizontalSlidesManual(double position)
    {
        horizontalPosition += position * 10;
    }

    public void HorizontalSlidesUpdate()
    {
        horizontalPosition = MathFunctions.clamp(horizontalPosition, 0, Constants.HORIZONTAL_SLIDES_MAX);

        double error = horizontalPosition - hardware.horizontalSlide.getCurrentPosition();

        double motorPower = error * Constants.HORIZONTAL_SLIDES_P_GAIN;
        motorPower = Math.min(Math.max(motorPower, -0.6), 0.6);

        hardware.horizontalSlide.setPower(motorPower);
    }

    public int GetHorizontalSlidePos()
    {
        return hardware.horizontalSlide.getCurrentPosition();
    }
}
