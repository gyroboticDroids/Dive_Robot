package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Master Tele-op", group = "Tele-op")
public class MasterTeleop extends OpMode {
    Drive drive;
    VerticalExtension vertExtension;

    @Override
    public void init()
    {
        drive = new Drive(hardwareMap, gamepad1);
        vertExtension = new VerticalExtension(hardwareMap);
    }

    @Override
    public void loop()
    {
        drive.Update();
        vertExtension.Update();

        if(!vertExtension.IsActive())
        {
            if(gamepad2.dpad_down)
            {
                vertExtension.SetState("reset");
            }
        }
    }
}
