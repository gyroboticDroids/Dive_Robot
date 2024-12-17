package org.firstinspires.ftc.teamcode.teleop;

import android.renderscript.ScriptGroup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.sql.Time;

public class VerticalExtension {
    private Hardware hardware;
    private Gamepad gamepad;

    private Timer sequenceTime;

    String state;

    public VerticalExtension (HardwareMap hardwareMap, Gamepad gamepad2)
    {
        hardware = new Hardware(hardwareMap);
        gamepad = gamepad2;

        sequenceTime = new Timer();

        SetState("reset");
    }

    public void Update()
    {
        switch (state)
        {
            case "reset":
                hardware.outtake.setPosition(Constants.OUTTAKE_CLOSED);
                VertSlidesControl(Constants.VERT_SLIDES_TRANSFER);
                break;
        }

    }
    public void SetState(String s)
    {
        sequenceTime.resetTimer();
        state = s;
        Update();
    }

    private void VertSlidesControl(double position)
    {
        double error = position - hardware.vertSlide1.getCurrentPosition();

        double motorPower = error * Constants.VERT_SLIDES_P_GAIN;
        motorPower = Math.min(Math.max(motorPower, -0.6), 0.6);

        hardware.vertSlide1.setPower(motorPower);
        hardware.vertSlide2.setPower(motorPower);
    }

    private void ExtensionControl()
    {

    }
}
