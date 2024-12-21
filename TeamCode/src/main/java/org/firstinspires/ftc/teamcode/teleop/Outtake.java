package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Outtake {
    private final Hardware hardware;

    String state;

    private final Timer actionTimer;

    public Outtake (HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);

        actionTimer = new Timer();
    }
    public void Update()
    {
        switch (state)
        {
            case "start":
                hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.pivot.setPosition(Constants.PIVOT_START);
                    hardware.wrist.setPosition(Constants.WRIST_START);
                }
                break;

            case "transfer intake ready":
                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.pivot.setPosition(Constants.PIVOT_TRANSFER);
                    hardware.wrist.setPosition(Constants.WRIST_TRANSFER);
                }

                if(actionTimer.getElapsedTimeSeconds() > 1)
                {
                    hardware.outtake.setPosition(Constants.OUTTAKE_OPEN);
                }
                else
                {
                    hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);
                }
                break;

            case "transfer intake":
                hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.pivot.setPosition(Constants.PIVOT_RAISE);
                    hardware.wrist.setPosition(Constants.WRIST_RAISE);
                }
                break;

            case "grab specimen ready":
                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.pivot.setPosition(Constants.PIVOT_OFF_WALL);
                    hardware.wrist.setPosition(Constants.WRIST_OFF_WALL);
                }

                if(actionTimer.getElapsedTimeSeconds() > 1)
                {
                    hardware.outtake.setPosition(Constants.OUTTAKE_OPEN);
                }
                else
                {
                    hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);
                }
                break;

            case "score specimen ready":
                hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);

                if(actionTimer.getElapsedTimeSeconds() > 0.5)
                {
                    hardware.pivot.setPosition(Constants.PIVOT_SPECIMEN);
                    hardware.wrist.setPosition(Constants.WRIST_SPECIMEN);
                }
                break;


            case "score sample":
                hardware.pivot.setPosition(Constants.PIVOT_SAMPLE);
                hardware.wrist.setPosition(Constants.WRIST_SAMPLE);

                if(actionTimer.getElapsedTimeSeconds() > 0.75)
                {
                    hardware.pivot.setPosition(Constants.OUTTAKE_OPEN);
                }
                break;
        }
    }

    public void SetState(String s)
    {
        actionTimer.resetTimer();
        state = s;
        Update();
    }

}
