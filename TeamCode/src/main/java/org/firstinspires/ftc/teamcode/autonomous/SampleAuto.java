package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.HangConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.teleop.Hang;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Outtake;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "4 sample auto", group = "autonomous", preselectTeleOp = "Master Tele-op")
public class SampleAuto extends OpMode {
    private static final int OUTTAKE_UP = 500;
    private int slideRange = 100;

    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;
    private Intake intake;
    private Outtake outtake;
    private Hang hang;
    private int pathState = -1;
    private int actionState = -1;
    private int nextPathState = 0;
    private boolean onsSetState;
    private boolean onsActionState;
    private boolean onsTimerState;
    private Path currentPath;

    private Path scorePreload, intoBucket, collectSampleRight, scoreSampleRight, collectSampleCenter, scoreSampleCenter, collectSampleLeft, scoreSampleLeft, touchBar;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_START), new Point(AutoConstants.SAMPLE_SCORE_READY)));
        scorePreload.setLinearHeadingInterpolation(AutoConstants.SAMPLE_START.getHeading(), AutoConstants.SAMPLE_SCORE_READY.getHeading());

        intoBucket = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_SCORE_READY), new Point(AutoConstants.SAMPLE_SCORE)));
        intoBucket.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE_READY.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading());

        collectSampleRight = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_SCORE), new Point(AutoConstants.SAMPLE_RIGHT)));
        collectSampleRight.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SAMPLE_RIGHT.getHeading());

        scoreSampleRight = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_RIGHT), new Point(AutoConstants.SAMPLE_SCORE_READY)));
        scoreSampleRight.setLinearHeadingInterpolation(AutoConstants.SAMPLE_RIGHT.getHeading(), AutoConstants.SAMPLE_SCORE_READY.getHeading());

        collectSampleCenter = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_SCORE), new Point(AutoConstants.SAMPLE_CENTER)));
        collectSampleCenter.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SAMPLE_CENTER.getHeading());

        scoreSampleCenter = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_CENTER), new Point(AutoConstants.SAMPLE_SCORE_READY)));
        scoreSampleCenter.setLinearHeadingInterpolation(AutoConstants.SAMPLE_CENTER.getHeading(), AutoConstants.SAMPLE_SCORE_READY.getHeading());

        collectSampleLeft = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_SCORE), new Point(AutoConstants.SAMPLE_LEFT)));
        collectSampleLeft.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SAMPLE_LEFT.getHeading());

        scoreSampleLeft = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_LEFT), new Point(AutoConstants.SAMPLE_SCORE_READY)));
        scoreSampleLeft.setLinearHeadingInterpolation(AutoConstants.SAMPLE_LEFT.getHeading(), AutoConstants.SAMPLE_SCORE_READY.getHeading());

        touchBar = new Path(new BezierCurve(new Point(AutoConstants.SAMPLE_SCORE), new Point(AutoConstants.SAMPLE_PARK.getX(), AutoConstants.SAMPLE_SCORE_READY.getY()), new Point(AutoConstants.SAMPLE_PARK)));
        touchBar.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SAMPLE_PARK.getHeading());
    }

    public void autonomousPathUpdate() {
        boolean robotInPos = MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getX(), follower.getPose().getX(), 1) &&
                MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getY(), follower.getPose().getY(), 1);

        switch (pathState) {
            case 0:
                if (onsSetState) {
                    setActionState(0);
                    intake.setState(IntakeConstants.RESET_POS);
                }

                if (!onsSetState ) {
                    currentPath = scorePreload;
                    follower.followPath(scorePreload, true);
                    nextPathState = 1;
                    setPathState(10);
                }
                break;

            case 1:
                if (robotInPos) {
                    if (actionState == -1) {
                        if (outtake.getState().equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
                            setActionState(1);
                        } else {
                            currentPath = collectSampleRight;
                            follower.followPath(currentPath, true);
                            setPathState(2);
                        }
                    }
                }
                break;

            case 2:
                if (robotInPos) {
                    if (onsActionState) {
                        setActionState(10);
                        onsActionState = false;
                    }
                    if ((actionState == -1 || actionState == 14)) {
                        currentPath = scoreSampleRight;
                        follower.followPath(currentPath, true);
                        nextPathState = 3;
                        setPathState(10);
                    }
                }
                break;

            case 3:
                if (robotInPos) {
                    if (actionState == -1) {
                        if (outtake.getState().equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
                            setActionState(1);
                        } else {
                            currentPath = collectSampleCenter;
                            follower.followPath(currentPath, true);
                            setPathState(4);
                        }
                    }
                }
                break;

            case 4:
                if (robotInPos) {
                    if (onsActionState) {
                        setActionState(10);
                        onsActionState = false;
                    }
                    if ((actionState == -1 || actionState == 14)) {
                        currentPath = scoreSampleCenter;
                        follower.followPath(currentPath, true);
                        nextPathState = 5;
                        setPathState(10);
                    }
                }
                break;

            case 5:
                if (robotInPos) {
                    if (actionState == -1) {
                        if (outtake.getState().equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
                            setActionState(1);
                        } else {
                            currentPath = collectSampleLeft;
                            follower.followPath(currentPath, true);
                            setPathState(6);
                        }
                    }
                }
                break;

            case 6:
                if (robotInPos) {
                    if (onsActionState) {
                        slideRange = 500;
                        setActionState(10);
                        onsActionState = false;
                    }
                    if ((actionState == -1 || actionState == 14)) {
                        currentPath = scoreSampleLeft;
                        follower.followPath(currentPath, true);
                        nextPathState = 7;
                        setPathState(10);
                    }
                }
                break;

            case 7:
                if (robotInPos) {
                    if (actionState == -1) {
                        if (outtake.getState().equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
                            setActionState(1);
                            hang.setState(HangConstants.TOUCH_BAR);
                        } else {
                            currentPath = touchBar;
                            follower.followPath(currentPath, true);
                            setActionState(20);
                            setPathState(8);
                        }
                    }
                }
                break;

            case 8:
                if (robotInPos) {
                    /*TODO Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;

            case 10:
                if (robotInPos && outtake.getVertSlidePos() > outtake.getVertPosition() - OUTTAKE_UP) {
                    currentPath = intoBucket;
                    follower.followPath(currentPath, true);
                    setPathState(nextPathState);
                }
                break;
        }
        onsSetState = false;
    }

    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                if (actionTimer.getElapsedTimeSeconds() > 0.25) {
                    outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
                    setActionState(14);
                }
                break;

            case 1:
                if (actionTimer.getElapsedTimeSeconds() > 0) {
                    outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                    setActionState(14);
                }
                break;

            case 10:
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                intake.setState(IntakeConstants.INTAKE_SUB_READY);
                setActionState(11);
                break;

            case 11:
                if (!intake.isBusy()) {
                    intake.setState(IntakeConstants.INTAKE);
                    intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX - slideRange);
                    setActionState(12);
                }
                break;

            case 12:
                if (intake.getHorizontalSlidePos() > IntakeConstants.SLIDES_MAX - slideRange - IntakeConstants.SLIDES_ACCURACY) {
                    if (onsTimerState) {
                        actionTimer.resetTimer();
                        onsTimerState = false;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                        intake.setState(IntakeConstants.TRANSFER);
                        setActionState(13);
                    }
                }
                break;

            case 13:
                if (!intake.isBusy()) {
                    if (onsTimerState) {
                        actionTimer.resetTimer();
                        onsTimerState = false;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.25) {
                        outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
                        setActionState(0);
                    }
                }
                break;

            case 14:
                if (!outtake.isBusy()) {
                    setActionState(-1);
                }
                break;

            case 20:
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        onsSetState = true;
        onsActionState = true;
        pathTimer.resetTimer();
    }

    public void setActionState(int aState) {
        actionState = aState;
        onsTimerState = true;
        actionTimer.resetTimer();
    }

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        intake.setState(IntakeConstants.START);

        outtake = new Outtake(hardwareMap);
        hang = new Hang(hardwareMap);
        hang.setState(HangConstants.START);

        pathTimer = new Timer();
        actionTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(AutoConstants.SAMPLE_START);
        follower.setMaxPower(0.5);
        buildPaths();

        currentPath = scorePreload;

        telemetry.addLine("initialized!");
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        outtake.update();
        intake.update();
        follower.update();

        autonomousPathUpdate();
        autonomousActionUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("action state", actionState);
        telemetry.addData("intake state", intake.getState());
        telemetry.addData("hori slide pos", intake.getHorizontalSlidePos());
        telemetry.addData("hori slide setpoint", intake.getHorizontalPosition());
        telemetry.addData("intake is busy", intake.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
