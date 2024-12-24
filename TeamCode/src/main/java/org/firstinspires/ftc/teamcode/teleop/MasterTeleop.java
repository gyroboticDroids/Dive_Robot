package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Master Tele-op", group = "Tele-op")
public class MasterTeleop extends OpMode {
    Drive drive;
    Outtake vertExtension;
    Hang hang;

    private boolean isHanging = false;

    @Override
    public void init()
    {
        drive = new Drive(hardwareMap, gamepad1);
        //vertExtension = new Outtake(hardwareMap);
        //hang = new Hang(hardwareMap);

        //hang.SetState("down");
    }

    @Override
    public void loop()
    {
        drive.Update();
        //VertExtensionUpdate();
        //HangUpdate();
    }

    void VertExtensionUpdate()
    {
        if(isHanging)
        {
            return;
        }

        drive.driveBack = vertExtension.IsDriveBack();

        vertExtension.Update();
        vertExtension.VertSlidesManual(gamepad2.left_stick_y);
    }

    void HangUpdate()
    {

    }
}
