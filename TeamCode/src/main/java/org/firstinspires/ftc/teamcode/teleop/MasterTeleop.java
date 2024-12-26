package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Master Tele-op", group = "Tele-op")
public class MasterTeleop extends OpMode {
    Drive drive;
    Outtake outtake;
    Intake intake;
    Hang hang;

    private boolean isHanging = false;

    @Override
    public void init()
    {
        drive = new Drive(hardwareMap, gamepad1);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        //hang = new Hang(hardwareMap);

        //hang.SetState("down");
    }

    @Override
    public void loop()
    {
        drive.Update();

        telemetry.addData("rx", drive.rx);
        telemetry.addData("target heading", drive.targetHeading);
        telemetry.addData("current heading", Math.toDegrees(drive.botHeading));
        telemetry.addData("vert slides pos", outtake.GetVertSlidePos());
        telemetry.addData("vert slides power", outtake.motorPower);
        OuttakeUpdate();
        //HangUpdate();
        telemetry.update();
    }

    void OuttakeUpdate()
    {
        //if(isHanging)
        //{
        //    return;
        //}

        //drive.driveBack = vertExtension.IsDriveBack();

        outtake.VertSlidesUpdate();
        outtake.VertSlidesManual(-gamepad2.left_stick_y);
    }

    void HangUpdate()
    {

    }
}
