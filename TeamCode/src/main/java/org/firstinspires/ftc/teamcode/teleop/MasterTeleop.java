package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Objects;

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
        IntakeUpdate();
        OuttakeUpdate();
        HangUpdate();
        telemetry.update();
    }

    String lastState;
    int yPresses = 0;
    boolean yIsPressed = false;

    void OuttakeUpdate()
    {
        if(isHanging)
        {
            return;
        }

        drive.driveBack = outtake.IsDriveBack();

        if(gamepad2.start)
        {
            outtake.SetState("start");
        }

        if(!Objects.equals(lastState, "transfer"))
        {
            if(gamepad2.a && Objects.equals(lastState, "sample score"))
            {
                outtake.SetState("score sample");
            }

            if(gamepad2.a && Objects.equals(lastState, "specimen score"))
            {
                outtake.SetState("score specimen");
            }
        }

        if(gamepad2.y && Objects.equals(lastState, "sample"))
        {
            if(yPresses < 2)
            {
                outtake.SetState("score sample ready high");
            }
            else
            {
                outtake.SetState("score sample ready low");
            }
        }

        if(gamepad2.y && Objects.equals(lastState, "specimen"))
        {
            if(yPresses < 2)
            {
                outtake.SetState("score specimen ready high");
            }
            else
            {
                outtake.SetState("score specimen ready low");
            }
        }

        if(gamepad2.y && !yIsPressed)
        {
            yIsPressed = true;
            yPresses++;
        }
        else
        {
            yIsPressed = false;
        }

        if(gamepad2.a)
        {
            yPresses = 0;
        }

        if(gamepad2.b)
        {
            outtake.SetState("grab specimen ready");
        }

        if(gamepad2.x)
        {
            outtake.SetState("transfer intake");
        }

        outtake.VertSlidesUpdate();
        outtake.VertSlidesManual(-gamepad2.left_stick_y);

        if(Objects.equals(outtake.state, "transfer intake"))
        {
            lastState = "sample";
        }
        else if (Objects.equals(outtake.state, "score sample ready high") || Objects.equals(outtake.state, "score sample ready low"))
        {
            lastState = "sample score";
        }
        else if (Objects.equals(outtake.state, "grab specimen ready"))
        {
            lastState = "specimen";
        }
        else if (Objects.equals(outtake.state, "score specimen ready high") || Objects.equals(outtake.state, "score specimen ready low"))
        {
            lastState = "specimen score";
        }
        else if (Objects.equals(outtake.state, "transfer intake ready"))
        {
            lastState = "transfer";
        }
    }

    void IntakeUpdate()
    {
        if (isHanging)
        {
            return;
        }

        if (gamepad2.back)
        {
            intake.SetState("start");
        }

        if (gamepad2.right_trigger > 0.1 || gamepad2.right_bumper)
        {
            intake.SetState("intake sub ready");
        }

        if (gamepad2.left_bumper)
        {
            intake.SetState("transfer");
        }

        if(!(Objects.equals(intake.state, "transfer") || Objects.equals(intake.state, "start")))
        {
            if (gamepad2.dpad_down)
            {
                intake.SetState("intake");
            }
            else if (gamepad2.dpad_up)
            {
                intake.SetState("reject");
            }
        }

        if(!(Objects.equals(intake.state, "transfer") || Objects.equals(intake.state, "start")))
        {
            intake.HorizontalSlidesManual(gamepad2.right_trigger - gamepad2.left_trigger);
        }

        intake.HorizontalSlidesUpdate();
    }

    void HangUpdate()
    {

    }
}
