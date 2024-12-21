package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class HorizontalExtension {

    private final Hardware hardware;

    private boolean isActive;

    private final Timer actionTimer;

    double horizontalPosition = 0;

    String state;

    public HorizontalExtension (HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);

        actionTimer = new Timer();

        SetState("start");
    }

    public void Update() {
        isActive = hardware.horizontalSlide.isBusy();

        switch (state) {
            case "start":
                hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_START);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    horizontalPosition(Constants.HORIZONTAL_SLIDES_START);
                }
                break;

            case "intake sub ready":
                horizontalPosition(Constants.HORIZONTAL_SLIDES_OUT);

                break;
            case "intake":

                break;
            case "transfer":

                break;
            case "reject":

                break;
        }
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

    public boolean IsActive()
    {
        return isActive;
    }
}
