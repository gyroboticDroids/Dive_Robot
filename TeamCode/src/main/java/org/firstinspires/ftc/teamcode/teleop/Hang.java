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
    private String lastState;

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
                hardware.hang1.setPosition(Constants.HANG1_DOWN);
                hardware.hang2.setPosition(Constants.HANG2_DOWN);
                break;

            case "hang ready":
                if(!Objects.equals(lastState, "hang ready"))
                {
                    outtake.vertPosition = Constants.VERT_SLIDES_HANG;
                }
                outtake.VertSlidesUpdate();

                hardware.hang1.setPosition(Constants.HANG1_UP);
                hardware.hang2.setPosition(Constants.HANG2_UP);
                break;

            case "lvl 2":
                hardware.hang1.setPosition(Constants.HANGING1);
                hardware.hang2.setPosition(Constants.HANGING2);

                if(hangTimer.getElapsedTimeSeconds() > 2)
                {
                    SetState("lvl 3");
                }
                break;

            case "lvl 3":
                hardware.hang1.setPosition(Constants.HANG1_DOWN);
                hardware.hang2.setPosition(Constants.HANG2_DOWN);

                if(!Objects.equals(lastState, "lvl 3"))
                {
                    outtake.vertPosition = Constants.VERT_SLIDES_START;
                }
                outtake.VertSlidesUpdate();
                break;
        }

        lastState = state;
    }

    public void SetState(String s) {
        hangTimer.resetTimer();
        state = s;
        Update();
    }
}