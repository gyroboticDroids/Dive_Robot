package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Master Tele-op", group = "Tele-op")
public class MasterTeleop extends OpMode {
    Drive drive;
    VerticalExtension vertExtension;
    Hang hang;

    private boolean isHanging = false;

    @Override
    public void init()
    {
        drive = new Drive(hardwareMap, gamepad1);
        vertExtension = new VerticalExtension(hardwareMap);
        hang = new Hang(hardwareMap);

        hang.SetState("down");
    }

    @Override
    public void loop()
    {
        drive.Update();
        VertExtensionUpdate();
        HangUpdate();
    }

    boolean isGrabbingSpecimen = false;
    boolean isTransferringSpecimen = false;

    void VertExtensionUpdate()
    {
        if(isHanging)
        {
            return;
        }

        drive.driveBack = vertExtension.IsDriveBack();

        vertExtension.Update();

    }

    void HangUpdate()
    {
        hang.Update();

        if(gamepad1.dpad_down)
        {
            isHanging = false;
            hang.SetState("down");
        }

        if(gamepad1.dpad_up)
        {
            isHanging = true;
            hang.SetState("up");
        }

        if(gamepad1.back)
        {
            isHanging = true;
            hang.SetState("hanging");
        }
    }
}
