package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.constants.HangConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Hang {
    private final Hardware hardware;
    private final Outtake outtake;

    String state;

    private boolean onsSetState = false;
    private boolean isBusy = false;

    private final Timer hangTimer;

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
            case HangConstants.START:
                hardware.hangRight.setPosition(HangConstants.RIGHT_DOWN);
                hardware.hangLeft.setPosition(HangConstants.LEFT_DOWN);

                if(hangTimer.getElapsedTimeSeconds() > 2)
                {
                    isBusy = false;
                }
                break;

            case HangConstants.HANG_READY:
                if(onsSetState)
                {
                    outtake.setVertPosition(OuttakeConstants.SLIDES_HANG);
                }
                outtake.VertSlidesUpdate();

                hardware.hangRight.setPosition(HangConstants.RIGHT_UP);
                hardware.hangLeft.setPosition(HangConstants.LEFT_UP);

                if(hangTimer.getElapsedTimeSeconds() > 2)
                {
                    isBusy = false;
                }
                break;

            case HangConstants.LVL_2:
                hardware.hangRight.setPosition(HangConstants.HANGING_RIGHT);
                hardware.hangLeft.setPosition(HangConstants.HANGING_LEFT);

                if(hangTimer.getElapsedTimeSeconds() > 2)
                {
                    isBusy = false;
                }
                break;

            case HangConstants.LVL_3:
                hardware.hangRight.setPosition(HangConstants.RIGHT_DOWN);
                hardware.hangLeft.setPosition(HangConstants.LEFT_DOWN);

                if(onsSetState)
                {
                    outtake.setVertPosition(OuttakeConstants.SLIDES_START);
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

    public boolean IsBusy()
    {
        return isBusy;
    }
}