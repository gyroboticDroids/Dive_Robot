package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class HorizontalExtension {

    private final Hardware hardware;

    private boolean isActive;

    private final Timer sequenceTime;

    double horizontalPosition = 0;

    String state;

    public HorizontalExtension (HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);

        sequenceTime = new Timer();

        SetState("start");
    }

    public void Update() {
        isActive = hardware.horizontalSlide.isBusy();

        switch (state) {
            case "start":
//                Intake Pivot Up
//
//                Horizontal Back
                hardware.intakePivot.setPosition(Constants.INTAKE_PIVOT_START);

        }
        switch (state) {
            case "intake sub ready":


        }
        switch (state) {
            case "intake":


        }
        switch (state) {
            case "transfer":


        }
        switch (state) {
            case "reject":


        }
    }
    public void SetState(String s)
    {
        sequenceTime.resetTimer();
        state = s;
        Update();
    }
    public void HorizontalSlidesUpdate()
    {
        double error = position - hardware.horizontalSlide.getCurrentPosition();

        double motorPower = error * Constants.HORIZONTAL_SLIDES_P_GAIN;
        motorPower = Math.min(Math.max(motorPower, -0.6), 0.6);

        hardware.horizontalSlide.setPower(motorPower);

    }

    public boolean IsActive()
    {
        return isActive;
    }
}
