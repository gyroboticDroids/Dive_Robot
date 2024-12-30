package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.Objects;

public class Hang {
    private final Hardware hardware;
    private final Outtake outtake;

    String state;
    private boolean onsSetState = false;
    private boolean isBusy = false;

    Timer hangTimer;

    public Hang(HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);
        outtake = new Outtake(hardwareMap);

        hangTimer = new Timer();
    }

    public void Update()
    {
        switch (state)
        {
            case "start":
                hardware.hangRight.setPosition(Constants.HANG_RIGHT_DOWN);
                hardware.hangLeft.setPosition(Constants.HANG_LEFT_DOWN);

                if(hangTimer.getElapsedTimeSeconds() > 2)
                {
                    isBusy = false;
                }
                break;

            case "hang ready":
                if(onsSetState)
                {
                    outtake.vertPosition = Constants.OUTTAKE_SLIDES_HANG;
                }
                outtake.VertSlidesUpdate();

                hardware.hangRight.setPosition(Constants.HANG_RIGHT_UP);
                hardware.hangLeft.setPosition(Constants.HANG_LEFT_UP);

                if(hangTimer.getElapsedTimeSeconds() > 2)
                {
                    isBusy = false;
                }
                break;

            case "lvl 2":
                hardware.hangRight.setPosition(Constants.HANGING_RIGHT);
                hardware.hangLeft.setPosition(Constants.HANGING_LEFT);

                if(hangTimer.getElapsedTimeSeconds() > 2)
                {
                    isBusy = false;
                }
                break;

            case "lvl 3":
                hardware.hangRight.setPosition(Constants.HANG_RIGHT_DOWN);
                hardware.hangLeft.setPosition(Constants.HANG_LEFT_DOWN);

                if(onsSetState)
                {
                    outtake.vertPosition = Constants.OUTTAKE_SLIDES_START;
                }
                outtake.VertSlidesUpdate();
                break;
        }

        onsSetState = false;
    }

    public void SetState(String s) {
        onsSetState = true;
        isBusy = true;
        hangTimer.resetTimer();
        state = s;
        Update();
    }
}