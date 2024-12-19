package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Outtake {
    private final Hardware hardware;

    String state;

    public Outtake (HardwareMap hardwareMap)
    {

        hardware = new Hardware(hardwareMap);

    }
    public void Update()
    {
        switch (state)
        {
            case "reset":
                hardware.outtake.setPosition(Constants.OUTTAKE_OPEN);
                hardware.pivot.setPosition(Constants.PIVOT_TRANSFER);
                hardware.wrist.setPosition(Constants.WRIST_TRANSFER);
                break;

            case "retract":
                hardware.outtake.setPosition(Constants.OUTTAKE_OPEN);
                hardware.pivot.setPosition(Constants.PIVOT_TRANSFER);
                hardware.wrist.setPosition(Constants.WRIST_TRANSFER);
                break;

            case "transfer":
                hardware.pivot.setPosition(Constants.PIVOT_TRANSFER);
                hardware.wrist.setPosition(Constants.WRIST_TRANSFER);
                break;

            case "grip":
                hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);
                break;

            case "score sample":
                hardware.pivot.setPosition(Constants.PIVOT_SAMPLE);
                hardware.wrist.setPosition(Constants.WRIST_SAMPLE);
                break;


            case "collect specimen":
                hardware.pivot.setPosition(Constants.PIVOT_OFF_WALL);
                hardware.wrist.setPosition(Constants.WRIST_OFF_WALL);
                break;

            case "transfer specimen":
                hardware.pivot.setPosition(Constants.PIVOT_SPECIMEN);
                hardware.wrist.setPosition(Constants.WRIST_SPECIMEN);
                break;

        }


    }

    public void SetState(String s)
    {
        state = s;
        Update();
    }

}
