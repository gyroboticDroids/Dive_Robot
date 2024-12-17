package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

public class Outtake {
    private Hardware hardware;
    private Gamepad gamepad;

    String state;

    public Outtake (HardwareMap hardwareMap, Gamepad gamepad2)
    {

        hardware = new Hardware(hardwareMap);
        gamepad = gamepad2;

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

            case "specimen":
                hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);
                hardware.pivot.setPosition(Constants.PIVOT_SPECIMEN);
                hardware.wrist.setPosition(Constants.WRIST_SPECIMEN);
                break;
        }
    }

}
