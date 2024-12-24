package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Hang {
    private final Hardware hardware;
    private final Outtake vertExtension;

    String state;

    Timer hangTimer;

    public Hang(HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);
        vertExtension = new Outtake(hardwareMap);

        hangTimer = new Timer();
    }

    public void Update()
    {
        switch (state)
        {
            case "down":
                hardware.hang1.setPosition(Constants.HANG1_DOWN);
                hardware.hang2.setPosition(Constants.HANG2_DOWN);
                break;

            case "up":
                hardware.hang1.setPosition(Constants.HANG1_UP);
                hardware.hang2.setPosition(Constants.HANG2_UP);
                break;

            case "hanging":
                hardware.hang1.setPosition(Constants.HANGING1);
                hardware.hang2.setPosition(Constants.HANGING2);

                if(hangTimer.getElapsedTimeSeconds() > 2)
                {
                    SetState("hanging lvl 3");
                }
                break;

            case "hanging lvl 3":
                hardware.hang1.setPosition(Constants.HANG1_DOWN);
                hardware.hang2.setPosition(Constants.HANG2_DOWN);
                break;
        }
    }

    public void SetState(String s) {
        hangTimer.resetTimer();
        state = s;
        Update();
    }
}