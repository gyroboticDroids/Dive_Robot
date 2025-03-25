package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.HangConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;

@TeleOp(name = "Master Tele-op", group = "Tele-op")
public class MasterTeleop extends OpMode {

    //Defines classes
    Drive drive;
    Outtake outtake;
    Intake intake;
    Hang hang;

    //Timer for automatic movements
    Timer teleopTimer;

    //Keeps track of if the robot is about to hang
    private boolean isHanging = false;

    private boolean hooksUp = false;

    private boolean rumble = false;

    Gamepad.RumbleEffect effect;

    @Override
    public void init()
    {
        //Sets up classes
        drive = new Drive(hardwareMap, gamepad1);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        intake.setIntakeWheelsKeepSpinning(true);
        hang = new Hang(hardwareMap, outtake);

        //Sets up timer
        teleopTimer = new Timer();

        effect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 500)
                .addStep(0.0, 0.0, 500)
                .addStep(0.5, 0.5, 500)
                .build();

        gamepad1.setLedColor(60.0 / 255, 76.0 / 255, 36.0 / 255, Gamepad.LED_DURATION_CONTINUOUS);
    }

    @Override
    public void stop()
    {
        TransferConstants.resetConstants();
    }

    @Override
    public void start()
    {
        //Resets timer
        teleopTimer.resetTimer();

        //Sets all states to the starting states
        outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
        intake.setState(IntakeConstants.INTAKE_SUB_READY);
        hang.setState(HangConstants.START);
    }

    @Override
    public void loop()
    {
        if(120 - teleopTimer.getElapsedTimeSeconds() < 30 && !hooksUp && !isHanging) {
            hang.setState(HangConstants.HANG_HOOKS_UP);

            gamepad1.rumble(0.5, 0.5, 500);
            gamepad2.rumble(0.5, 0.5, 500);

            hooksUp = true;
        }

        if(120 - teleopTimer.getElapsedTimeSeconds() < 15 && !rumble) {
            hang.setState(HangConstants.HANG_HOOKS_UP);

            gamepad1.runRumbleEffect(effect);
            gamepad2.runRumbleEffect(effect);

            rumble = true;
        }

        //Updates all classes
        if(!(hang.getState().equals(HangConstants.LVL_2) || hang.getState().equals(HangConstants.LVL_3))){
            drive.update();
        }
        else {
            drive.resetPowers();
        }

        outtakeUpdate();
        intakeUpdate();
        hangUpdate();
        autoUpdate();

        //Updates classes
        outtake.update();
        intake.update();
        hang.update();

        //Updates telemetry
        updateTelemetry();
    }

    private void updateTelemetry() {
        //Time remaining in the match
        telemetry.addData("time remaining", 120 - teleopTimer.getElapsedTimeSeconds());
        telemetry.addLine();

        //Telemetry
        telemetry.addLine("-------------------Drive---------------------");
        telemetry.addData("target heading", drive.targetHeading);
        telemetry.addData("current heading", Math.toDegrees(drive.getBotHeading()));
        telemetry.addData("is manual turning", drive.isManualTurning());
        telemetry.addData("turn power", drive.getTurnPower());

        telemetry.addLine("-------------------Outtake-------------------");
        telemetry.addData("outtake state", outtake.getState());
        telemetry.addData("outtake busy", outtake.isBusy());
        telemetry.addData("vert spt pos", outtake.getVertPosition());
        telemetry.addData("vert fdbk pos", outtake.getVertSlidePos());
        telemetry.addData("drive back", outtake.isDriveBack());
        telemetry.addData("is hanging", outtake.isHanging());
        telemetry.addData("slide power", outtake.getSlidePower());
        telemetry.addData("disable reject", disableReject);

        telemetry.addLine("-------------------Intake--------------------");
        telemetry.addData("intake state", intake.getState());
        telemetry.addData("intake busy", intake.isBusy());
        telemetry.addData("horizontal spt pos", intake.getHorizontalPosition());
        telemetry.addData("horizontal fdbk pos", intake.getHorizontalSlidePos());
        telemetry.addData("horizontal power", intake.getMotorPower());
        telemetry.addData("sample color", intake.getSampleColor());
        telemetry.addData("intake speed", intake.getIntakeSpeed());

        telemetry.addLine("-------------------Hang----------------------");
        telemetry.addData("hang state", hang.getState());
        telemetry.addData("hang busy", hang.isBusy());
        telemetry.addData("hanging", isHanging);

        telemetry.addLine("-------------------Auto----------------------");

        //Updates telemetry
        telemetry.update();
    }

    //Variables used in outtakeUpdate class
    private String prevOuttakeState = OuttakeConstants.START;
    private boolean prevGp2Y = false;
    private boolean prevGp2B = false;
    private boolean disableReject = false;
    private boolean prevGp1Start = false;

    void outtakeUpdate()
    {
        //Stops the outtake from interrupting hanging
        if(isHanging)
        {
            return;
        }

        //If outtake is busy then don't receive any inputs
        if(!outtake.isBusy())
        {
            //All buttons and statements that change outtake states
            if (gamepad2.start && !(prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE))) {
                outtake.setState(OuttakeConstants.START);
            } else if (gamepad2.back && (prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY) || prevOuttakeState.equals(OuttakeConstants.START) || prevOuttakeState.equals(OuttakeConstants.GRAB_SPECIMEN_READY))) {
                outtake.setState(OuttakeConstants.RESET_POS);
            } else if ((gamepad2.x || gamepad2.b) && prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY) && intake.getState().equals(IntakeConstants.TRANSFER) && !intake.isBusy()) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
            } else if (prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE) && intake.getSampleColor() > 0 && !disableReject && outtake.isSlidesAtSetpoint()) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY); ///If transfer did not work run this code
            } else if (gamepad2.b && (prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE) || prevOuttakeState.equals(OuttakeConstants.START))) {
                outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
            } else if (gamepad2.b && !prevGp2B && prevOuttakeState.equals(OuttakeConstants.GRAB_SPECIMEN_READY) && prevIntakeState.equals(IntakeConstants.TRANSFER)) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
            } else if (gamepad2.b && !prevGp2B && (prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH))) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
            } else if (gamepad2.a && prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN)) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH); ///If misses scoring lower pivot
            } else if (gamepad2.b && !prevGp2B && prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN)) {
                outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
            } else if (gamepad2.x && prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE)) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
            }  else if (gamepad2.a && (prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW))) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE);
            } else if (gamepad2.x && (prevOuttakeState.equals(OuttakeConstants.GRAB_SPECIMEN_READY) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN)) ||
                    ((gamepad2.x || gamepad2.b) && prevOuttakeState.equals(OuttakeConstants.START)) || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE) || prevOuttakeState.equals(OuttakeConstants.RESET_POS)) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
            }
        }

        if(prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW)
                || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.START)
                || prevOuttakeState.equals(OuttakeConstants.GRAB_SPECIMEN_READY)
                || prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY)) {
            outtake.vertSlidesManual(-gamepad2.left_stick_y * 2); //Manual control for vert slides
        }

        if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
            outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_LOW);
        } else if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW)) {
            outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
        }

        //Makes robot drive back if collecting specimens off wall or scoring samples
        if(outtake.isDriveBack()) {
            drive.driveBack(DriveConstants.DRIVE_BACK_POWER);
        } else {
            drive.driveBack(0);
        }

        if(gamepad1.start && !prevGp1Start && !disableReject) {
            disableReject = true;
        } else if (gamepad1.start && !prevGp1Start && disableReject) {
            disableReject = false;
        }

        //Previous states and button presses
        prevOuttakeState = outtake.getState();
        prevGp2Y = gamepad2.y;
        prevGp2B = gamepad2.b;
        prevGp1Start = gamepad1.start;
    }

    //Variables used for intakeUpdate
    private String prevIntakeState = IntakeConstants.START;

    void intakeUpdate()
    {
        //Stops the intake from interrupting hanging
        if (isHanging)
        {
            intake.horizontalSlidesUpdate();
            return;
        }

        //If outtake is busy then don't receive any inputs
        if(!intake.isBusy())
        {
            //All buttons and statements that change outtake states
            if (gamepad2.start) {
                intake.setState(IntakeConstants.START);
            } else if (gamepad2.back && (prevIntakeState.equals(IntakeConstants.TRANSFER) || prevIntakeState.equals(IntakeConstants.START))) {
                    intake.setState(IntakeConstants.RESET_POS);
            } else if (prevIntakeState.equals(IntakeConstants.RESET_POS)) {
                intake.setState(IntakeConstants.TRANSFER);
            } else if (gamepad2.left_bumper || (gamepad2.b || gamepad2.x) && (prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY) || prevOuttakeState.equals(OuttakeConstants.GRAB_SPECIMEN_READY)) && !prevIntakeState.equals(IntakeConstants.TRANSFER)) {
                intake.setState(IntakeConstants.TRANSFER);
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);/*Outtake*/
            } else if ((gamepad2.right_bumper || gamepad1.right_bumper) && (prevIntakeState.equals(IntakeConstants.TRANSFER) || prevIntakeState.equals(IntakeConstants.START) || prevIntakeState.equals(IntakeConstants.HALFWAY))) {
                intake.setState(IntakeConstants.INTAKE_SUB_READY);
            } else if (gamepad2.dpad_down && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.REJECT)
                    || prevIntakeState.equals(IntakeConstants.CLEAR_SUB))) {
                intake.setState(IntakeConstants.INTAKE);
            } else if (gamepad2.dpad_up && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.INTAKE)
                    || prevIntakeState.equals(IntakeConstants.CLEAR_SUB))) {
                intake.setState(IntakeConstants.REJECT);
            } else if (gamepad2.dpad_left && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.INTAKE)
                    || prevIntakeState.equals(IntakeConstants.REJECT))) {
                intake.setState(IntakeConstants.CLEAR_SUB);
            }
        }
        else if(gamepad2.dpad_right && !prevIntakeState.equals(IntakeConstants.RESET_POS)){
            intake.setState(IntakeConstants.HALFWAY);
        }

        if (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.REJECT)) {
            intake.horizontalSlidesManual((MathFunctions.clamp(gamepad2.right_trigger + ((gamepad1.right_bumper)?1:0), 0, 1) -
                    MathFunctions.clamp(gamepad2.left_trigger + ((gamepad1.left_bumper)?1:0), 0, 1)) * 100); //Manual control for horizontal slides
        } else if (prevIntakeState.equals(IntakeConstants.INTAKE) || prevIntakeState.equals(IntakeConstants.CLEAR_SUB)) {
            intake.horizontalSlidesManual((MathFunctions.clamp(gamepad2.right_trigger + ((gamepad1.right_bumper)?1:0), 0, 1) -
                    MathFunctions.clamp(gamepad2.left_trigger + ((gamepad1.left_bumper)?1:0), 0, 1)) * 50); //Manual control for horizontal slides
        }

        if (intake.getSampleColor() == 1) {
            gamepad2.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (intake.getSampleColor() == 2) {
            gamepad2.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (intake.getSampleColor() == 3) {
            gamepad2.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad2.setLedColor(1, 1, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }

        //Previous intake state
        prevIntakeState = intake.getState();
    }

    void hangUpdate()
    {
        //Sets if the robot is hanging
        isHanging = !(hang.getState().equals(HangConstants.START) || hang.getState().equals(HangConstants.HANG_HOOKS_UP));

        //Doesn't change states if the hang is busy
        if(!hang.isBusy())
        {
            //All buttons and statements that change hang states
            if (gamepad1.dpad_down) {
                hang.setState(HangConstants.START);
            } else if (gamepad1.dpad_up) {
                hang.setState(HangConstants.HANG_READY);
                intake.setState(IntakeConstants.START);
            } else if (gamepad1.start && hang.getState().equals(HangConstants.HANG_READY)) {
                hang.setState(HangConstants.LVL_2);
            } else if (gamepad1.start && hang.getState().equals(HangConstants.LVL_2)) {
                hang.setState(HangConstants.LVL_3);
            }
        }
    }

    void autoUpdate()
    {
        if(isHanging) {
            return;
        }

    }
}
