package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class HorizontalExtension {

    private final Hardware hardware;
    private final Timer actionTimer;

    private String lastState;

    double horizontalPosition = 0;

    String state;

    public HorizontalExtension (HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);

        actionTimer = new Timer();

        SetState("start");
    }

    public void Update() {

        switch (state) {
            case "start":
                hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_START);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    horizontalPosition = Constants.HORIZONTAL_SLIDES_START;

                }
                break;

            case "intake sub ready":

                horizontalPosition = Constants.HORIZONTAL_SLIDES_OUT;
                if(horizontalPosition > Constants.HORIZONTAL_SLIDES_TRANSFER)
                {
                    hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_INTERMEDIATE);
                }
                break;
            case "intake":

                hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_DOWN);
                break;

            case "transfer":

                hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_TRANSFER);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    horizontalPosition = Constants.HORIZONTAL_SLIDES_START;
                }
                break;

            case "reject":
                hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_INTERMEDIATE);

                break;
        }

        lastState = state;
    }

    public void Intake()
    {
        hardware.intake1.setPower(Constants.INTAKE_SPEED);
        hardware.intake2.setPower(Constants.INTAKE_SPEED);
    }

    public void SetState(String s)
    {
        actionTimer.resetTimer();
        state = s;
        Update();
    }

    public void HorizontalSlidesUpdate()
    {
        double error = horizontalPosition - hardware.horizontalSlide.getCurrentPosition();

        double motorPower = error * Constants.HORIZONTAL_SLIDES_P_GAIN;
        motorPower = Math.min(Math.max(motorPower, -0.6), 0.6);

        hardware.horizontalSlide.setPower(motorPower);

    }
}
