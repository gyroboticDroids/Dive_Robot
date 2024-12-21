package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class HorizontalExtension {

    private final Hardware hardware;

    private boolean isActive;

    private final Timer sequenceTime;

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
}
