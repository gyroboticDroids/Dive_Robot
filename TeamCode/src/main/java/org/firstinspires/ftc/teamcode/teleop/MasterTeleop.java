package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.HangConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@TeleOp(name = "Master Tele-op", group = "Tele-op")
public class MasterTeleop extends OpMode {

    Drive drive;
    Outtake outtake;
    Intake intake;
    Hang hang;

    Timer teleopTimer;

    private boolean isHanging = false;

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
        hang.setState(HangConstants.START);
    }

    @Override
    public void loop()
    {
        drive.Update();
        OuttakeUpdate();
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
        telemetry.addData("hang state", hang.getState());
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
            if (gamepad2.start && prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY)) {
                outtake.setState(OuttakeConstants.START);
            } else if (gamepad2.x && !prevGp2X && !(prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN)
                    || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE))) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
            } else if (gamepad2.x && !prevGp2X && prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY)) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
            } else if (gamepad2.b && (prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY) || prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE)
                    || prevOuttakeState.equals(OuttakeConstants.START))) {
                outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
            } else if (gamepad2.y && !prevGp2Y && (prevOuttakeState.equals(OuttakeConstants.GRAB_SPECIMEN_READY) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_LOW))) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
            } else if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH)) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_LOW);
            } else if (gamepad2.y && !prevGp2Y && (prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE) || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW))) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
            } else if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_LOW);
            } else if (gamepad2.a && (prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW))) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE);
            } else if (gamepad2.a && (prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_LOW))) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
            } else if (prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN)) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
            } else if (prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW)
                    || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_LOW)
                    || prevOuttakeState.equals(OuttakeConstants.START)) {
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
        intake.HorizontalSlidesUpdate();

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
            } else if (gamepad2.right_bumper && (prevIntakeState.equals(IntakeConstants.TRANSFER)
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
        isHanging = !hang.getState().equals(HangConstants.START);

        if(!hang.IsBusy())
        {
            if (gamepad1.dpad_down) {
                hang.setState(HangConstants.START);
            } else if (gamepad1.dpad_up && prevOuttakeState.equals(OuttakeConstants.START) && prevIntakeState.equals(IntakeConstants.START)) {
                hang.setState(HangConstants.HANG_READY);
            } else if (gamepad1.start && hang.getState().equals(HangConstants.HANG_READY)) {
                hang.setState(HangConstants.LVL_2);
            } else if (hang.getState().equals(HangConstants.LVL_2)) {
                hang.setState(HangConstants.LVL_3);
            }
        }

        hang.Update();
    }
}
