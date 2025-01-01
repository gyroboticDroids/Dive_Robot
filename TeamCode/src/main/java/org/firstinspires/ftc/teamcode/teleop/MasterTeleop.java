package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.HangConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;

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

        outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
        intake.setState(IntakeConstants.TRANSFER);
        hang.setState(HangConstants.START);
    }

    @Override
    public void loop()
    {
        drive.update();
        outtakeUpdate();
        intakeUpdate();
        hangUpdate();

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
        telemetry.addData("outtake busy", outtake.isBusy());
        telemetry.addData("vert spt pos", outtake.getVertPosition());
        telemetry.addData("vert fdbk pos", outtake.getVertSlidePos());
        telemetry.addData("drive back", outtake.isDriveBack());

        telemetry.addLine("-------------------Intake--------------------");
        telemetry.addData("intake state", intake.getState());
        telemetry.addData("intake busy", intake.isBusy());
        telemetry.addData("horizontal spt pos", intake.getHorizontalPosition());
        telemetry.addData("horizontal fdbk pos", intake.getHorizontalSlidePos());
        telemetry.addData("horizontal power", intake.getMotorPower());

        telemetry.addLine("-------------------Hang----------------------");
        telemetry.addData("hang state", hang.getState());
        telemetry.addData("hang busy", hang.isBusy());
        telemetry.addData("hanging", isHanging);

        telemetry.update();
    }

    private String prevOuttakeState = OuttakeConstants.START;
    private boolean prevGp2Y = false;
    private boolean prevGp2X = false;

    void outtakeUpdate()
    {
        if(isHanging)
        {
            return; //Doesn't interrupt hanging
        }

        if(!outtake.isBusy())
        {
            if (gamepad2.start && prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY)) {
                outtake.setState(OuttakeConstants.START);
            } else if (gamepad2.x && !prevGp2X && !(prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN)
                    || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE))) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
            } else if (gamepad2.x && !prevGp2X && prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY)) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
            } else if (gamepad2.b && (prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY) || prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE)
                    || prevOuttakeState.equals(OuttakeConstants.START) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_LOW))) {
                outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
            } else if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.GRAB_SPECIMEN_READY)) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
            } else if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE)) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
            }  else if (gamepad2.a && (prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW))) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE);
            } else if (gamepad2.a && (prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_LOW))) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
            } else if (prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN)) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
            } else if (prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW)
                    || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_LOW)
                    || prevOuttakeState.equals(OuttakeConstants.START)) {
                outtake.vertSlidesManual(-gamepad2.left_stick_y);
            }
        }
        else
        {
            if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH)) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_LOW);
            } else if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_LOW)) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
            } else if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_LOW);
            } else if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW)) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
            }
        }

        outtake.update();

        drive.setDriveBack(outtake.isDriveBack());

        prevOuttakeState = outtake.getState();
        prevGp2X = gamepad2.x;
        prevGp2Y = gamepad2.y;
    }

    private String prevIntakeState = IntakeConstants.START;

    void intakeUpdate()
    {
        if (isHanging)
        {
            return;
        }

        if(!intake.isBusy())
        {
            if (gamepad2.start && (prevIntakeState.equals(IntakeConstants.TRANSFER) || prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY))) {
                intake.setState(IntakeConstants.START);
            } else if (gamepad2.back && prevIntakeState.equals(IntakeConstants.TRANSFER)) {
                    intake.setState(IntakeConstants.RESET_POS);
            } else if (gamepad2.left_bumper && prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY) && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.START)
                    || prevIntakeState.equals(IntakeConstants.INTAKE) || prevIntakeState.equals(IntakeConstants.REJECT) || prevIntakeState.equals(IntakeConstants.CLEAR_SUB))) {
                intake.setState(IntakeConstants.TRANSFER);
            } else if (gamepad2.right_bumper || gamepad1.right_bumper && (prevIntakeState.equals(IntakeConstants.TRANSFER)
                    || prevIntakeState.equals(IntakeConstants.START) || prevIntakeState.equals(IntakeConstants.INTAKE) || prevIntakeState.equals(IntakeConstants.CLEAR_SUB))) {
                intake.setState(IntakeConstants.INTAKE_SUB_READY);
            } else if (gamepad2.dpad_down && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.REJECT)
                    || prevIntakeState.equals(IntakeConstants.CLEAR_SUB))) {
                intake.setState(IntakeConstants.INTAKE);
            } else if (gamepad2.dpad_up && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.INTAKE)
                    || prevIntakeState.equals(IntakeConstants.CLEAR_SUB))) {
                intake.setState(IntakeConstants.REJECT);
            } else if ((gamepad2.dpad_right || gamepad2.dpad_left) && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.INTAKE)
                    || prevIntakeState.equals(IntakeConstants.REJECT))) {
                intake.setState(IntakeConstants.CLEAR_SUB);
            } else if (prevIntakeState.equals(IntakeConstants.RESET_POS)) {
                intake.setState(IntakeConstants.TRANSFER);
            } else if (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.INTAKE)
                    || prevIntakeState.equals(IntakeConstants.REJECT) || prevIntakeState.equals(IntakeConstants.CLEAR_SUB)) {
                intake.horizontalSlidesManual((MathFunctions.clamp(gamepad2.right_trigger + ((gamepad1.right_bumper)?1:0), 0, 1) -
                MathFunctions.clamp(gamepad2.left_trigger + ((gamepad1.left_bumper)?1:0), 0, 1)) * 10);
            }
        }

        intake.update();

        prevIntakeState = intake.getState();
    }

    void hangUpdate()
    {
        isHanging = !hang.getState().equals(HangConstants.START);

        if(!hang.isBusy())
        {
            if (gamepad1.dpad_down) {
                hang.setState(HangConstants.START);
            } else if (gamepad1.dpad_up && prevOuttakeState.equals(OuttakeConstants.START) && prevIntakeState.equals(IntakeConstants.START)) {
                hang.setState(HangConstants.HANG_READY);
            } else if (gamepad1.start && hang.getState().equals(HangConstants.HANG_READY)) {
                hang.setState(HangConstants.LVL_2);
            } //else if (hang.getState().equals(HangConstants.LVL_2)) {
            //    hang.setState(HangConstants.LVL_3);
            //}
        }

        hang.update();
    }
}
