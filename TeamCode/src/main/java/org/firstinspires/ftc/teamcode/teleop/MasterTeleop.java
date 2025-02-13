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

    //Defines classes
    Drive drive;
    Outtake outtake;
    Intake intake;
    Hang hang;

    //Timer for automatic movements
    Timer teleopTimer;

    //Keeps track of if the robot is about to hang
    private boolean isHanging = false;

    @Override
    public void init()
    {
        //Sets up classes
        drive = new Drive(hardwareMap, gamepad1);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        intake.setIntakeWheelsKeepSpinning(false);
        hang = new Hang(hardwareMap);

        //Sets up timer
        teleopTimer = new Timer();
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

        //Updates telemetry
        telemetry.update();
    }

    //Variables used in outtakeUpdate class
    private String prevOuttakeState = OuttakeConstants.START;
    private boolean prevGp2Y = false;

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
            if (gamepad2.start && prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY)) {
                outtake.setState(OuttakeConstants.START);
            } else if (gamepad2.back && (prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY) || prevOuttakeState.equals(OuttakeConstants.START))) {
                outtake.setState(OuttakeConstants.RESET_POS);
            } else if ((gamepad2.x || gamepad2.b) && prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY) && intake.getState().equals(IntakeConstants.TRANSFER) && !intake.isBusy()) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
            } else if (gamepad2.b && (prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE) || prevOuttakeState.equals(OuttakeConstants.START))) {
                outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
            } else if (gamepad2.b && prevOuttakeState.equals(OuttakeConstants.GRAB_SPECIMEN_READY)) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
            } else if (gamepad2.x && prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE)) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
            }  else if (gamepad2.a && (prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW))) {
                outtake.setState(OuttakeConstants.SCORE_SAMPLE);
            } else if (gamepad2.a && (prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_LOW))) {
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
            } else if ((gamepad2.y && prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE)) || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE) || prevOuttakeState.equals(OuttakeConstants.RESET_POS) ||
                    ((gamepad2.x || gamepad2.b) && prevOuttakeState.equals(OuttakeConstants.START))) {
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
            } else if (prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN)) {
                outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
            } else if (prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW)
                    || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH) || prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_LOW)
                    || prevOuttakeState.equals(OuttakeConstants.START)) {
                outtake.vertSlidesManual(-gamepad2.left_stick_y); //Manual control for vert slides
            }
        } else if (prevOuttakeState.equals(OuttakeConstants.GRAB_SPECIMEN_READY) || prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY)) {
            outtake.vertSlidesManual(-gamepad2.left_stick_y); //Manual control for vert slides
        }

        //Control for states that can run while the outtake is busy
        if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH)) {
            outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_LOW);
        } else if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_LOW)) {
            outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
        } else if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
            outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_LOW);
        } else if (gamepad2.y && !prevGp2Y && prevOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_LOW)) {
            outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
        }

        //Updates outtake
        outtake.update();

        //Makes robot drive back if collecting specimens off wall or scoring samples
        drive.setDriveBack(outtake.isDriveBack());

        //Previous states and button presses
        prevOuttakeState = outtake.getState();
        prevGp2Y = gamepad2.y;
    }

    //Variables used for intakeUpdate
    private String prevIntakeState = IntakeConstants.START;

    void intakeUpdate()
    {
        //Stops the intake from interrupting hanging
        if (isHanging)
        {
            return;
        }

        //If outtake is busy then don't receive any inputs
        if(!intake.isBusy())
        {
            //All buttons and statements that change outtake states
            if (gamepad2.start && (prevIntakeState.equals(IntakeConstants.TRANSFER) || prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY))) {
                intake.setState(IntakeConstants.START);
            } else if (gamepad2.back && prevIntakeState.equals(IntakeConstants.TRANSFER)) {
                    intake.setState(IntakeConstants.RESET_POS);
            } else if (prevIntakeState.equals(IntakeConstants.RESET_POS)) {
                intake.setState(IntakeConstants.TRANSFER);
            } else if (gamepad2.left_bumper || (gamepad2.b || gamepad2.x) && prevOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY) && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.START)
                    || prevIntakeState.equals(IntakeConstants.INTAKE) || prevIntakeState.equals(IntakeConstants.REJECT) || prevIntakeState.equals(IntakeConstants.CLEAR_SUB) || prevIntakeState.equals(IntakeConstants.HALFWAY))) {
                intake.setState(IntakeConstants.TRANSFER);
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);/*Outtake*/
            } else if ((gamepad2.right_bumper || gamepad1.right_bumper) && (prevIntakeState.equals(IntakeConstants.TRANSFER)
                    || prevIntakeState.equals(IntakeConstants.START) || prevIntakeState.equals(IntakeConstants.HALFWAY))) {
                intake.setState(IntakeConstants.INTAKE_SUB_READY);
                if(outtake.getState().equals(OuttakeConstants.GRAB_SPECIMEN_READY) && !outtake.isBusy()){
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);/*Outtake*/
                }
            } else if (gamepad2.dpad_down && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.REJECT)
                    || prevIntakeState.equals(IntakeConstants.CLEAR_SUB))) {
                intake.setState(IntakeConstants.INTAKE);
            } else if (gamepad2.dpad_up && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.INTAKE)
                    || prevIntakeState.equals(IntakeConstants.CLEAR_SUB))) {
                intake.setState(IntakeConstants.REJECT);
            } else if (gamepad2.dpad_left && (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.INTAKE)
                    || prevIntakeState.equals(IntakeConstants.REJECT))) {
                intake.setState(IntakeConstants.CLEAR_SUB);
            } else if (prevIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) || prevIntakeState.equals(IntakeConstants.INTAKE)
                    || prevIntakeState.equals(IntakeConstants.REJECT) || prevIntakeState.equals(IntakeConstants.CLEAR_SUB)) {
                intake.horizontalSlidesManual((MathFunctions.clamp(gamepad2.right_trigger + ((gamepad1.right_bumper)?1:0), 0, 1) -
                    MathFunctions.clamp(gamepad2.left_trigger + ((gamepad1.left_bumper)?1:0), 0, 1)) * 40); //Manual control for horizontal slides
            }
        }
        else if(gamepad2.dpad_right && !prevIntakeState.equals(IntakeConstants.RESET_POS)){
            intake.setState(IntakeConstants.HALFWAY);
        }

        //Updates intake
        intake.update();

        //Previous intake state
        prevIntakeState = intake.getState();
    }

    private boolean prevGp1Start = false;

    void hangUpdate()
    {
        //Sets if the robot is hanging
        isHanging = !hang.getState().equals(HangConstants.START);

        //Doesn't change states if the hang is busy
        if(!hang.isBusy())
        {
            //All buttons and statements that change hang states
            if (gamepad1.dpad_down) {
                hang.setState(HangConstants.START);
            } else if (gamepad1.dpad_up && prevOuttakeState.equals(OuttakeConstants.START) && prevIntakeState.equals(IntakeConstants.START) || hang.getState().equals(HangConstants.LVL_3)) {
                hang.setState(HangConstants.HANG_READY);
            } else if (gamepad1.start && hang.getState().equals(HangConstants.HANG_READY)) {
                hang.setState(HangConstants.LVL_2);
            } else if (gamepad1.start && !prevGp1Start && hang.getState().equals(HangConstants.LVL_2)) {
                hang.setState(HangConstants.LVL_3);
            }
        }

        //Updates the hang
        hang.update();
        prevGp1Start = gamepad1.start;
    }
}
