package org.firstinspires.ftc.teamcode.teleop;

import android.renderscript.ScriptGroup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

public class VerticalExtension {
    private Hardware hardware;
    private Gamepad gamepad;

    String state;

    public VerticalExtension (HardwareMap hardwareMap, Gamepad gamepad2)
    {

        hardware = new Hardware(hardwareMap);
        gamepad = gamepad2;



    }

    public void Update()
    {
        switch (state)
        {
            case "reset":
                hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);
                VertSlidesPosition(Constants.VERT_SLIDES_TRANSFER);
                break;
        }

    }
    public void SetState(String s)
    {
        state = s;
        Update();
    }

    public void VertSlidesPosition(double position)
    {



    }


}
