package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Master Tele-op", group = "Tele-op")
public class MasterTeleop extends OpMode {
    Drive drive;

    @Override
    public void init()
    {
        drive = new Drive();
        drive.Configure(hardwareMap);
    }

    @Override
    public void loop()
    {
        drive.Update(gamepad1);
    }
}
