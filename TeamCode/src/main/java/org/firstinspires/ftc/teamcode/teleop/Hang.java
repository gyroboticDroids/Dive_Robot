package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.constants.HangConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;

public class Hang {
    private final Hardware hardware;
    private final Outtake outtake;

    private String state;

    private boolean onsSetState = false;
    private boolean isBusy = false;

    private final Timer hangTimer;

    public Hang(HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);
        outtake = new Outtake(hardwareMap);

        hangTimer = new Timer();
    }

    public void update()
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
                outtake.vertSlidesUpdate();

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
                    outtake.setVertPosition(OuttakeConstants.SLIDES_START + 1);
                }
                outtake.vertSlidesUpdate();
                break;

            case HangConstants.TOUCH_BAR:
                hardware.hangRight.setPosition(HangConstants.RIGHT_UP);
                break;
        }
        onsSetState = false;
    }

    public String getState() {
        return state;
    }

    public void setState(String s) {
        onsSetState = true;
        isBusy = true;
        hangTimer.resetTimer();
        state = s;
        //Update();
    }

    public boolean isBusy()
    {
        return isBusy;
    }
}