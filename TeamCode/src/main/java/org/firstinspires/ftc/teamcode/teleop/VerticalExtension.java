package org.firstinspires.ftc.teamcode.teleop;

import android.renderscript.ScriptGroup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

public class VerticalExtension {
    private Hardware hardware;

    public void configure (HardwareMap hardwareMap)
    {

        hardware = new Hardware();
        hardware.ConfigureHardware(hardwareMap);

    }

    public void Update()
    {



    }

}
