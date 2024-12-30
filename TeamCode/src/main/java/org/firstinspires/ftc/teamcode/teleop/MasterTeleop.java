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
    }

    @Override
    public void start()
    {
        teleopTimer.resetTimer();

        outtake.SetState("start");
        intake.SetState("start");
        hang.SetState("start");
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
        telemetry.addData("outtake busy", outtake.IsBusy());
        telemetry.addData("vert spt pos", outtake.vertPosition);
        telemetry.addData("vert fdbk pos", outtake.GetVertSlidePos());
        telemetry.addData("drive back", outtake.IsDriveBack());

        telemetry.addLine("-------------------Intake--------------------");
        telemetry.addData("intake state", intake.state);
        telemetry.addData("intake busy", intake.IsBusy());
        telemetry.addData("horizontal spt pos", intake.horizontalPosition);
        telemetry.addData("horizontal fdbk pos", intake.GetHorizontalSlidePos());

        telemetry.addLine("-------------------Hang----------------------");
        telemetry.addData("hang state", hang.state);
        telemetry.addData("hang busy", hang.IsBusy());
        telemetry.addData("hanging", isHanging);

        telemetry.update();
    }

    String prevOuttakeState;
    boolean prevGp2Y = false;
    boolean prevGp2X = false;

    void OuttakeUpdate()
    {
        if(isHanging)
        {
            return;
        }

        if(outtake.IsBusy())
        {
            if (gamepad2.start && Objects.equals(prevOuttakeState, "transfer intake ready")) {
                outtake.SetState("start");
            } else if (gamepad2.x && !prevGp2X && (Objects.equals(prevOuttakeState, "start") || Objects.equals(prevOuttakeState, "grab specimen ready"))) {
                outtake.SetState("transfer intake ready");
            } else if (gamepad2.x && !prevGp2X && Objects.equals(prevOuttakeState, "transfer intake ready")) {
                outtake.SetState("transfer intake");
            } else if (gamepad2.b && (Objects.equals(prevOuttakeState, "transfer intake ready") || Objects.equals(prevOuttakeState, "transfer intake")
                    || Objects.equals(prevOuttakeState, "start"))) {
                outtake.SetState("grab specimen ready");
            } else if (gamepad2.y && !prevGp2Y && (Objects.equals(prevOuttakeState, "grab specimen ready") || Objects.equals(prevOuttakeState, "score specimen ready low"))) {
                outtake.SetState("score specimen ready high");
            } else if (gamepad2.y && !prevGp2Y && Objects.equals(prevOuttakeState, "score specimen ready high")) {
                outtake.SetState("score specimen ready low");
            } else if (gamepad2.y && !prevGp2Y && (Objects.equals(prevOuttakeState, "transfer intake") || Objects.equals(prevOuttakeState, "score sample ready low"))) {
                outtake.SetState("score sample ready high");
            } else if (gamepad2.y && !prevGp2Y && Objects.equals(prevOuttakeState, "score sample ready high")) {
                outtake.SetState("score sample ready low");
            } else if (gamepad2.a && (Objects.equals(prevOuttakeState, "score sample ready high") || Objects.equals(prevOuttakeState, "score sample ready low"))) {
                outtake.SetState("score sample");
            } else if (gamepad2.a && (Objects.equals(prevOuttakeState, "score specimen ready high") || Objects.equals(prevOuttakeState, "score specimen ready low"))) {
                outtake.SetState("score specimen");
            } else if (Objects.equals(prevOuttakeState, "score sample") || Objects.equals(prevOuttakeState, "score specimen")) {
                outtake.SetState("transfer intake ready");
            }
        }

        if (Objects.equals(prevOuttakeState, "score sample ready high") || Objects.equals(prevOuttakeState, "score sample ready low")
                || Objects.equals(prevOuttakeState, "score specimen ready high") || Objects.equals(prevOuttakeState, "score specimen ready low")
                || Objects.equals(prevOuttakeState, "start")) {
            outtake.VertSlidesManual(-gamepad2.left_stick_y);
        }

        outtake.Update();
        outtake.VertSlidesUpdate();

        drive.driveBack = outtake.IsDriveBack();

        prevOuttakeState = outtake.state;
        prevGp2X = gamepad2.x;
        prevGp2Y = gamepad2.y;
    }

    String prevIntakeState;

    void IntakeUpdate()
    {
        if (isHanging)
        {
            return;
        }

        if(intake.IsBusy())
        {
            if (gamepad2.back && Objects.equals(prevIntakeState, "transfer")) {
                intake.SetState("start");
            } else if (gamepad2.left_bumper && (Objects.equals(prevIntakeState, "intake sub ready") || Objects.equals(prevIntakeState, "start")
                    || Objects.equals(prevIntakeState, "intake"))) {
                intake.SetState("transfer");
            } else if ((gamepad2.right_trigger > 0.1 || gamepad2.right_bumper) && (Objects.equals(prevIntakeState, "transfer")
                    || Objects.equals(prevIntakeState, "start") || Objects.equals(prevIntakeState, "intake"))) {
                intake.SetState("intake sub ready");
            } else if (gamepad2.dpad_down && (Objects.equals(prevIntakeState, "intake sub ready") || Objects.equals(prevIntakeState, "reject"))) {
                intake.SetState("intake");
            } else if (gamepad2.dpad_up && (Objects.equals(prevIntakeState, "intake sub ready") || Objects.equals(prevIntakeState, "intake"))) {
                intake.SetState("reject");
            }
        }

        if (Objects.equals(prevIntakeState, "intake sub ready") || Objects.equals(prevIntakeState, "intake")
                || Objects.equals(prevIntakeState, "reject")) {
            intake.HorizontalSlidesManual(gamepad2.right_trigger - gamepad2.left_trigger);
        }

        intake.Update();
        intake.HorizontalSlidesUpdate();

        prevIntakeState = intake.state;
    }

    void HangUpdate()
    {
        isHanging = !Objects.equals(hang.state, "start");

        if(hang.IsBusy())
        {
            if (gamepad1.dpad_down) {
                hang.SetState("start");
            } else if (gamepad1.dpad_up && Objects.equals(prevOuttakeState, "start") && Objects.equals(prevIntakeState, "start")) {
                hang.SetState("hang ready");
            } else if (gamepad1.start && Objects.equals(hang.state, "hang ready")) {
                hang.SetState("lvl 2");
            } else if (Objects.equals(hang.state, "lvl 2")) {
                hang.SetState("lvl 3");
            }
        }

        hang.Update();
    }
}
