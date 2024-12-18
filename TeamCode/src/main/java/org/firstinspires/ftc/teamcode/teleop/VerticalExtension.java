package org.firstinspires.ftc.teamcode.teleop;

import android.renderscript.ScriptGroup;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.sql.Time;

public class VerticalExtension {
    private final Hardware hardware;

    private boolean isActive;

    private Timer sequenceTime;

    String state;

    public VerticalExtension (HardwareMap hardwareMap)
    {
        hardware = new Hardware(hardwareMap);

        sequenceTime = new Timer();

        SetState("reset");
    }

    public void Update()
    {
        isActive = hardware.vertSlide1.isBusy();

        switch (state)
        {
            case "reset":
                //TODO: close gripper
                //TODO: add outtake class

                hardware.specimenExtension.setPosition(Constants.EXTENSION_BACK);
                VertSlidesControl(Constants.VERT_SLIDES_TRANSFER);
                break;

            case "retract":
                //TODO: open gripper and move arm back

                if(sequenceTime.getElapsedTimeSeconds() > 0.5)
                {
                    SetState("reset");
                }
                break;

            case "transfer":
                //TODO: open
                //TODO: add arm moving down

                if(sequenceTime.getElapsedTimeSeconds() > 0.5)
                {
                    SetState("grip");
                }
                break;

            case "grip":
                //TODO: grab and raise
                break;

            case "score sample":
                VertSlidesControl(Constants.VERT_SLIDES_SAMPLE);
                //TODO: add arm moving over

                break;

            case "collect specimen":
                //TODO: open

                hardware.specimenExtension.setPosition(Constants.EXTENSION_BACK);
                VertSlidesControl(Constants.VERT_SLIDES_TRANSFER);
                break;

            case "grab specimen":
                //TODO: grab

                if(sequenceTime.getElapsedTimeSeconds() > 0.5)
                {
                    SetState("transfer specimen");
                }
                break;

            case "transfer specimen":
                //TODO: Add outtake movement here

                hardware.specimenExtension.setPosition(Constants.EXTENSION_SPECIMEN);
                VertSlidesControl(Constants.VERT_SLIDES_SPECIMEN);
                break;

            case "score specimen":
                VertSlidesControl(Constants.VERT_SLIDES_SPECIMEN_SCORING);

                if(sequenceTime.getElapsedTimeSeconds() > 0.5)
                {
                    SetState("retract");
                }
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

    public boolean IsActive()
    {
        return isActive;
    }
}
