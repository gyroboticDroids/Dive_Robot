package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.Objects;

@TeleOp(name = "Master Tele-op", group = "Tele-op")
public class MasterTeleop extends OpMode {
    Drive drive;
    Outtake outtake;
    Intake intake;
    Hang hang;

    Timer teleopTimer;

    private boolean isHanging = false;
    private boolean isRumble = false;

    @Override
    public void init()
    {
        drive = new Drive(hardwareMap, gamepad1);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        hang = new Hang(hardwareMap);

        teleopTimer = new Timer();

        hang.SetState("start");
    }

    @Override
    public void start()
    {
        teleopTimer.resetTimer();
    }

    @Override
    public void loop()
    {
        if(teleopTimer.getElapsedTimeSeconds() >= 90 && !isRumble)
        {
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);

            isRumble = true;
        }

        drive.Update();
        OuttakeUpdate();
        IntakeUpdate();
        HangUpdate();

        telemetry.addData("time remaining", 120 - teleopTimer.getElapsedTimeSeconds());
        telemetry.addLine();

        telemetry.addLine("-------------------Drive---------------------");
        telemetry.addData("target heading", drive.targetHeading);
        telemetry.addData("current heading", Math.toDegrees(drive.botHeading));

        telemetry.addLine("-------------------Outtake-------------------");
        telemetry.addData("outtake state", outtake.state);
        telemetry.addData("vert setpoint pos", outtake.vertPosition);
        telemetry.addData("vert fdbk pos", outtake.GetVertSlidePos());
        telemetry.addData("drive back", outtake.IsDriveBack());

        telemetry.addLine("-------------------Intake--------------------");
        telemetry.addData("intake state", intake.state);
        telemetry.addData("horizontal setpoint pos", intake.horizontalPosition);
        telemetry.addData("horizontal fdbk pos", intake.GetHorizontalSlidePos());

        telemetry.addLine("-------------------Hang----------------------");
        telemetry.addData("hang state", hang.state);
        telemetry.addData("hanging", isHanging);

        telemetry.update();
    }

    String lastOuttakeState;
    boolean yIsPressed = false;
    boolean xIsPressed = false;

    void OuttakeUpdate()
    {
        if(isHanging)
        {
            return;
        }

        if(gamepad2.start && Objects.equals(lastOuttakeState, "transfer intake ready"))
        {
            outtake.SetState("start");
        }
        else if (gamepad2.x && !xIsPressed && (Objects.equals(lastOuttakeState, "start") || Objects.equals(lastOuttakeState, "grab specimen ready")))
        {
            outtake.SetState("transfer intake ready");
            xIsPressed = true;
        }
        else if (gamepad2.x && !xIsPressed && Objects.equals(lastOuttakeState, "transfer intake ready"))
        {
            outtake.SetState("transfer intake");
            xIsPressed = true;
        }
        else if(!gamepad2.x)
        {
            xIsPressed = false;
        }
        else if (gamepad2.b && (Objects.equals(lastOuttakeState, "transfer intake ready") || Objects.equals(lastOuttakeState, "transfer intake")
                || Objects.equals(lastOuttakeState, "start")))
        {
            outtake.SetState("grab specimen ready");
        }
        else if(gamepad2.y && !yIsPressed && (Objects.equals(lastOuttakeState, "grab specimen ready") || Objects.equals(lastOuttakeState, "score specimen ready low")))
        {
            outtake.SetState("score specimen ready high");
            yIsPressed = true;
        }
        else if(gamepad2.y && !yIsPressed && Objects.equals(lastOuttakeState, "score specimen ready high"))
        {
            outtake.SetState("score specimen ready low");
            yIsPressed = true;
        }
        else if(gamepad2.y && !yIsPressed && (Objects.equals(lastOuttakeState, "transfer intake") || Objects.equals(lastOuttakeState, "score sample ready low")))
        {
            outtake.SetState("score sample ready high");
            yIsPressed = true;
        }
        else if(gamepad2.y && !yIsPressed && Objects.equals(lastOuttakeState, "score sample ready high"))
        {
            outtake.SetState("score sample ready low");
            yIsPressed = true;
        }
        else if(!gamepad2.y)
        {
            yIsPressed = false;
        }
        else if(gamepad2.a && (Objects.equals(lastOuttakeState, "score sample ready high") || Objects.equals(lastOuttakeState, "score sample ready low")))
        {
            outtake.SetState("score sample");
        }
        else if(gamepad2.a && (Objects.equals(lastOuttakeState, "score specimen ready high") || Objects.equals(lastOuttakeState, "score specimen ready low")))
        {
            outtake.SetState("score score specimen");
        }
        else if(Objects.equals(lastOuttakeState, "score sample ready high") || Objects.equals(lastOuttakeState, "score sample ready low")
                || Objects.equals(lastOuttakeState, "score specimen ready high") || Objects.equals(lastOuttakeState, "score specimen ready low")
                || Objects.equals(lastOuttakeState, "start"))
        {
            outtake.VertSlidesManual(-gamepad2.left_stick_y);
        }

        outtake.Update();
        outtake.VertSlidesUpdate();

        drive.driveBack = outtake.IsDriveBack();

        lastOuttakeState = outtake.state;
    }

    String lastIntakeState;

    void IntakeUpdate()
    {
        if (isHanging)
        {
            return;
        }

        if (gamepad2.back && Objects.equals(lastIntakeState, "transfer"))
        {
            intake.SetState("start");
        }
        else if (gamepad2.left_bumper && (Objects.equals(lastIntakeState, "intake sub ready") || Objects.equals(lastIntakeState, "start")
                || Objects.equals(lastIntakeState, "intake")))
        {
            intake.SetState("transfer");
        }
        else if ((gamepad2.right_trigger > 0.1 || gamepad2.right_bumper) && (Objects.equals(lastIntakeState, "transfer")
                || Objects.equals(lastIntakeState, "start") || Objects.equals(lastIntakeState, "intake")))
        {
            intake.SetState("intake sub ready");
        }
        else if (gamepad2.dpad_down && (Objects.equals(lastIntakeState, "intake sub ready") || Objects.equals(lastIntakeState, "reject")))
        {
            intake.SetState("intake");
        }
        else if (gamepad2.dpad_up && (Objects.equals(lastIntakeState, "intake sub ready") || Objects.equals(lastIntakeState, "intake")))
        {
            intake.SetState("reject");
        }
        else if (Objects.equals(lastIntakeState, "intake sub ready") || Objects.equals(lastIntakeState, "intake")
                || Objects.equals(lastIntakeState, "reject"))
        {
            intake.HorizontalSlidesManual(gamepad2.right_trigger - gamepad2.left_trigger);
        }

        intake.Update();
        intake.HorizontalSlidesUpdate();

        lastIntakeState = intake.state;
    }

    void HangUpdate()
    {
        isHanging = !Objects.equals(hang.state, "start");

        if(gamepad1.dpad_down)
        {
            hang.SetState("start");
        }
        else if (gamepad1.dpad_up && Objects.equals(lastOuttakeState, "start") && Objects.equals(lastIntakeState, "start"))
        {
            hang.SetState("hang ready");
        }
        else if(gamepad1.start && Objects.equals(hang.state, "hang ready"))
        {
            hang.SetState("lvl 2");
        }
    }
}
