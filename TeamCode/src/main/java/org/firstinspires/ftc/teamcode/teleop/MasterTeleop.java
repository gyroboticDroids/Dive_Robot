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
        VertExtensionUpdate();
    }

    boolean isGrabbingSpecimen = false;
    boolean isTransferringSpecimen = false;

    void VertExtensionUpdate()
    {
        vertExtension.Update();

        isTransferringSpecimen = vertExtension.state == "transfer specimen";

        if(gamepad2.dpad_down && !vertExtension.IsActive())
        {
            vertExtension.SetState("retract");
        }

        if(gamepad2.a && vertExtension.state == "reset" && !isGrabbingSpecimen && !isTransferringSpecimen)
        {
            vertExtension.SetState("collect specimen");
        }

        if(gamepad2.a && isGrabbingSpecimen)
        {
            vertExtension.SetState("grab specimen");
        }

        if(gamepad2.a && isTransferringSpecimen)
        {
            vertExtension.SetState("score specimen");
        }

        if(gamepad2.b && vertExtension.state == "reset")
        {
            vertExtension.SetState("transfer");
        }

        if(gamepad2.b && vertExtension.state == "grip")
        {
            vertExtension.SetState("score sample");
        }

        isGrabbingSpecimen = vertExtension.state == "collect specimen" && !gamepad2.a;
    }
}
