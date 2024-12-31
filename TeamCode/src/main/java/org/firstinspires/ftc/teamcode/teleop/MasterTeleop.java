package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.HangConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.Objects;

@TeleOp(name = "Master Tele-op", group = "Tele-op")
public class MasterTeleop extends OpMode {
    private static final int RUMBLE = 1000;

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

        outtake.setState(OuttakeConstants.START);
        intake.setState(IntakeConstants.START);
        hang.SetState(HangConstants.START);
    }

    @Override
    public void loop()
    {
        if(teleopTimer.getElapsedTimeSeconds() >= 90 && !isRumble)
        {
            gamepad1.rumble(RUMBLE);
            gamepad2.rumble(RUMBLE);

            isRumble = true;
        }

        drive.Update();
        //OuttakeUpdate();
        IntakeUpdate();
        HangUpdate();

        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("time remaining", 120 - teleopTimer.getElapsedTimeSeconds());
        telemetry.addLine();

        telemetry.addLine("-------------------Drive---------------------");
        telemetry.addData("target heading", drive.targetHeading);
        telemetry.addData("current heading", Math.toDegrees(drive.botHeading));

        telemetry.addLine("-------------------Outtake-------------------");
        telemetry.addData("outtake state", outtake.getState());
        telemetry.addData("outtake busy", outtake.IsBusy());
        telemetry.addData("vert spt pos", outtake.getVertPosition());
        telemetry.addData("vert fdbk pos", outtake.GetVertSlidePos());
        telemetry.addData("drive back", outtake.IsDriveBack());

        telemetry.addLine("-------------------Intake--------------------");
        telemetry.addData("intake state", intake.getState());
        telemetry.addData("intake busy", intake.IsBusy());
        telemetry.addData("horizontal spt pos", intake.getHorizontalPosition());
        telemetry.addData("horizontal fdbk pos", intake.GetHorizontalSlidePos());
        telemetry.addData("horizontal power", intake.getMotorPower());

        telemetry.addLine("-------------------Hang----------------------");
        telemetry.addData("hang state", hang.state);
        telemetry.addData("hang busy", hang.IsBusy());
        telemetry.addData("hanging", isHanging);

        telemetry.update();
    }

    private String prevOuttakeState = OuttakeConstants.START;
    private boolean prevGp2Y = false;
    private boolean prevGp2X = false;

    void OuttakeUpdate()
    {
        if(isHanging)
        {
            return; //Doesn't interrupt hanging
        }

        if(!outtake.IsBusy())
        {
            if (gamepad2.start && prevIntakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY)) {
                outtake.setState(OuttakeConstants.START);
            } else if (gamepad2.x && !prevGp2X && (prevIntakeState.equals(OuttakeConstants.START) || prevIntakeState.equals(OuttakeConstants.GRAB_SPECIMEN_READY))) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
            } else if (gamepad2.x && !prevGp2X && prevIntakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY)) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
            } else if (gamepad2.b && (prevIntakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY) || prevIntakeState.equals(OuttakeConstants.TRANSFER_INTAKE)
                    || prevIntakeState.equals(OuttakeConstants.START))) {
                outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
            } else if (gamepad2.y && !prevGp2Y && (prevIntakeState.equals(OuttakeConstants.GRAB_SPECIMEN_READY) || prevIntakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_LOW))) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
            } else if (gamepad2.y && !prevGp2Y && prevIntakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH)) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_LOW);
            } else if (gamepad2.y && !prevGp2Y && (prevIntakeState.equals(OuttakeConstants.TRANSFER_INTAKE) || prevIntakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW))) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
            } else if (gamepad2.y && !prevGp2Y && prevIntakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_LOW);
            } else if (gamepad2.a && (prevIntakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH) || prevIntakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW))) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE);
            } else if (gamepad2.a && (prevIntakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH) || prevIntakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_LOW))) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
            } else if (prevIntakeState.equals(OuttakeConstants.SCORE_SAMPLE) || prevIntakeState.equals(OuttakeConstants.SCORE_SPECIMEN)) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
            } else if (prevIntakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH) || prevIntakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW)
                    || prevIntakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH) || prevIntakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_LOW)
                    || prevIntakeState.equals(OuttakeConstants.START)) {
                outtake.VertSlidesManual(-gamepad2.left_stick_y);
            }
        }

        outtake.Update();

        drive.setDriveBack(outtake.IsDriveBack());

        prevOuttakeState = outtake.getState();
        prevGp2X = gamepad2.x;
        prevGp2Y = gamepad2.y;
    }

    private String prevIntakeState = IntakeConstants.START;

    void IntakeUpdate()
    {
        if (isHanging)
        {
            return;
        }

        if(!intake.IsBusy())
        {
            if (gamepad2.back && prevIntakeState.equals(IntakeConstants.TRANSFER)) {
                intake.setState(IntakeConstants.START);
            } else if (gamepad2.left_bumper && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.START)
                    || prevIntakeState.equals(IntakeConstants.INTAKE))) {
                intake.setState(IntakeConstants.TRANSFER);
            } else if ((gamepad2.right_trigger > 0.1 || gamepad2.right_bumper) && (prevIntakeState.equals(IntakeConstants.TRANSFER)
                    || prevIntakeState.equals(IntakeConstants.START) || prevIntakeState.equals(IntakeConstants.INTAKE))) {
                intake.setState(IntakeConstants.INTAKE_SUB_READY);
            } else if (gamepad2.dpad_down && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.REJECT))) {
                intake.setState(IntakeConstants.INTAKE);
            } else if (gamepad2.dpad_up && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.INTAKE))) {
                intake.setState(IntakeConstants.REJECT);
            } else if (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.INTAKE)
                    || prevIntakeState.equals(IntakeConstants.REJECT)) {
                intake.HorizontalSlidesManual((gamepad2.right_trigger - gamepad2.left_trigger) * 5);
            }
        }

        intake.Update();

        prevIntakeState = intake.getState();
    }

    void HangUpdate()
    {
        isHanging = !hang.state.equals(HangConstants.START);

        if(!hang.IsBusy())
        {
            if (gamepad1.dpad_down) {
                hang.SetState(HangConstants.START);
            } else if (gamepad1.dpad_up && prevOuttakeState.equals(OuttakeConstants.START) && prevIntakeState.equals(IntakeConstants.START)) {
                hang.SetState(HangConstants.HANG_READY);
            } else if (gamepad1.start && hang.state.equals(HangConstants.HANG_READY)) {
                hang.SetState(HangConstants.LVL_2);
            } else if (hang.state.equals(HangConstants.LVL_2)) {
                hang.SetState(HangConstants.LVL_3);
            }
        }

        hang.Update();
    }
}
