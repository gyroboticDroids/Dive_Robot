package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

public class Hang {
    private final Hardware hardware;

    String state;


    public Hang(HardwareMap hardwareMap) {

        hardware = new Hardware(hardwareMap);

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

            case "hanging":
                hardware.hang1.setPosition(Constants.HANGING1);
                hardware.hang2.setPosition(Constants.HANGING2);
                break;
        }
    }

    public void SetState(String s) {
        state = s;
        Update();
    }
}