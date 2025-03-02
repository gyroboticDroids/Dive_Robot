package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.pathgen.MathFunctions;
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

    public Hang(HardwareMap hardwareMap, Outtake out)
    {
        hardware = new Hardware(hardwareMap);
        outtake = out;

        hangTimer = new Timer();
    }

    public void update()
    {
        switch (state)
        {
            case HangConstants.START:
                hardware.hangRight.setPosition(HangConstants.RIGHT_DOWN);
                hardware.hangLeft.setPosition(HangConstants.LEFT_DOWN);

                outtake.setHanging(false);

                if(hangTimer.getElapsedTimeSeconds() > 2)
                {
                    isBusy = false;
                }
                break;

            case HangConstants.HANG_HOOKS_UP:
                hardware.hangRight.setPosition(HangConstants.RIGHT_UP);
                hardware.hangLeft.setPosition(HangConstants.LEFT_UP);

                if(hangTimer.getElapsedTimeSeconds() > 2)
                {
                    isBusy = false;
                }
                break;

            case HangConstants.HANG_READY:
                outtake.setHanging(true);

                if(onsSetState)
                {
                    outtake.setState(OuttakeConstants.START);
                }

                hardware.hangRight.setPosition(HangConstants.RIGHT_UP);
                hardware.hangLeft.setPosition(HangConstants.LEFT_UP);

                if(MathFunctions.roughlyEquals(outtake.getVertSlidePos(), outtake.getVertPosition(), OuttakeConstants.SLIDES_ACCURACY)) {
                    isBusy = false;
                }
                break;

            case HangConstants.LVL_2:
                hardware.hangRight.setPosition(HangConstants.HANGING_RIGHT);
                hardware.hangLeft.setPosition(HangConstants.HANGING_LEFT);

                if(hangTimer.getElapsedTimeSeconds() > 1)
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
                break;

            case HangConstants.TOUCH_BAR:
                hardware.hangRight.setPosition(HangConstants.RIGHT_TOUCH_BAR);
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