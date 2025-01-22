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

    private int slideRangeSubtract = 100;

    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;
    private Intake intake;
    private Outtake outtake;
    private Hang hang;
    private int pathState = -1;
    private int actionState = -1;
    private boolean onsTimerState;
    private boolean onsIntakeState;
    private boolean onsScoreState;
    private boolean onsMoveState;
    private Path currentPath;
    private double currentHeading;
    private boolean robotInPos;
    private boolean intakeReady = true;

    private Path scorePreload, collectSampleRight, scoreSampleRight, collectSampleCenter, scoreSampleCenter, collectSampleLeft, scoreSampleLeft,
            collectSampleSub, scoreSampleSub, touchBar;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_START), new Point(AutoConstants.SAMPLE_SCORE_RIGHT)));
        scorePreload.setLinearHeadingInterpolation(AutoConstants.SAMPLE_START.getHeading(), AutoConstants.SAMPLE_SCORE_RIGHT.getHeading(), 0.6);

        collectSampleRight = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_SCORE_RIGHT), new Point(AutoConstants.SAMPLE_RIGHT)));
        collectSampleRight.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE_RIGHT.getHeading(), AutoConstants.SAMPLE_RIGHT.getHeading(), 0.8);

        scoreSampleRight = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_RIGHT), new Point(AutoConstants.SAMPLE_SCORE_CENTER)));
        scoreSampleRight.setLinearHeadingInterpolation(AutoConstants.SAMPLE_RIGHT.getHeading(), AutoConstants.SAMPLE_SCORE_CENTER.getHeading(), 0.8);

        collectSampleCenter = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_SCORE_CENTER), new Point(AutoConstants.SAMPLE_CENTER)));
        collectSampleCenter.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE_CENTER.getHeading(), AutoConstants.SAMPLE_CENTER.getHeading(), 0.8);

        scoreSampleCenter = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_CENTER), new Point(AutoConstants.SAMPLE_SCORE_LEFT)));
        scoreSampleCenter.setLinearHeadingInterpolation(AutoConstants.SAMPLE_CENTER.getHeading(), AutoConstants.SAMPLE_SCORE_LEFT.getHeading(), 0.8);

        collectSampleLeft = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_SCORE_LEFT), new Point(AutoConstants.SAMPLE_LEFT)));
        collectSampleLeft.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE_LEFT.getHeading(), AutoConstants.SAMPLE_LEFT.getHeading(), 0.8);

        scoreSampleLeft = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_LEFT), new Point(AutoConstants.SAMPLE_SCORE_LEFT)));
        scoreSampleLeft.setLinearHeadingInterpolation(AutoConstants.SAMPLE_LEFT.getHeading(), AutoConstants.SAMPLE_SCORE_LEFT.getHeading(), 0.8);

        collectSampleSub = new Path(new BezierCurve(new Point(AutoConstants.SAMPLE_SCORE_LEFT), new Point(AutoConstants.SAMPLE_SUB.getX(), AutoConstants.SAMPLE_SCORE.getY()), new Point(AutoConstants.SAMPLE_SUB)));
        collectSampleSub.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE_LEFT.getHeading(), AutoConstants.SAMPLE_SUB.getHeading());

        scoreSampleSub = new Path(new BezierCurve(new Point(AutoConstants.SAMPLE_SUB), new Point(AutoConstants.SAMPLE_SUB.getX(), AutoConstants.SAMPLE_SCORE.getY()), new Point(AutoConstants.SAMPLE_SCORE)));
        scoreSampleSub.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SUB.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading());

        touchBar = new Path(new BezierCurve(new Point(AutoConstants.SAMPLE_SCORE), new Point(AutoConstants.SAMPLE_PARK.getX(), AutoConstants.SAMPLE_SCORE_LEFT.getY()), new Point(AutoConstants.SAMPLE_PARK)));
        touchBar.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SAMPLE_PARK.getHeading());
    }

    public void autonomousPathUpdate() {
        robotInPos = MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getX(), follower.getPose().getX(), 1) &&
                MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getY(), follower.getPose().getY(), 1) &&
                MathFunctions.roughlyEquals(currentHeading, follower.getPose().getHeading(), Math.toRadians(5));

        switch (pathState) {
            case 0:
                intake.setState(IntakeConstants.RESET_POS);
                setActionState(0);

                currentPath = scorePreload;
                currentHeading = currentPath.getHeadingGoal(1);
                follower.followPath(currentPath, true);
                setPathState(1);
                break;

            case 1:
                if (robotInPos) {
                    if (actionState == -1) {
                        if(onsScoreState) {
                            setActionState(5);
                            onsScoreState = false;
                        }
                        else {
                            currentPath = scoreSampleRight;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            setActionState(0);
                            setPathState(2);
                        }
                    } else if (actionState == 7) {
                        if(onsMoveState) {
                            currentPath = collectSampleRight;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            onsMoveState = false;
                        }
                    }
                }
                break;

            case 2:
                if (robotInPos) {
                    if (actionState == -1) {
                        if(onsScoreState) {
                            setActionState(5);
                            onsScoreState = false;
                        }
                        else {
                            currentPath = scoreSampleCenter;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            setActionState(0);
                            setPathState(3);
                        }
                    }else if (actionState == 7) {
                        if(onsMoveState) {
                            currentPath = collectSampleCenter;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            onsMoveState = false;
                        }
                    }
                }
                break;

            case 3:
                if (robotInPos) {
                    if (actionState == -1) {
                        if(onsScoreState) {
                            setActionState(5);
                            intakeReady = false;
                            onsScoreState = false;
                        }
                        else {
                            currentPath = scoreSampleLeft;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            setActionState(0);
                            setPathState(4);
                        }
                    }else if (outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE_READY)) {
                        if(onsMoveState) {
                            currentPath = collectSampleLeft;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            onsMoveState = false;
                        }
                        else {
                            intakeReady = true;
                        }
                    }
                }
                break;

            case 4:
                if (robotInPos) {
                    if (actionState == -1) {
                        if(onsScoreState) {
                            setActionState(10);
                            hang.setState(HangConstants.TOUCH_BAR);
                            onsScoreState = false;
                        }
                    }else if (actionState == 11) {
                        if(onsMoveState) {
                            currentPath = collectSampleSub;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            setPathState(5);
                        }
                    }
                }
                break;

            case 5:
                if (robotInPos) {
                    if (actionState == -1) {
                        if(onsScoreState) {
                            setActionState(15);
                            onsScoreState = false;
                        }
                    }else if (actionState == 17) {
                        if(onsMoveState) {
                            currentPath = scoreSampleSub;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            setPathState(6);
                        }
                    }
                }
                break;

            case 6:
                if (robotInPos) {
                    if (actionState == -1) {
                        if(onsScoreState){
                            setActionState(10);
                            onsScoreState = false;
                        }else {
                            currentPath = touchBar;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            setPathState(10);
                        }
                    }
                }
                break;

            case 10:
                if (robotInPos) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);

                if(pathState < 5) {
                    intake.setState(IntakeConstants.INTAKE_SUB_READY);
                    setActionState(13);
                } else {
                    setActionState(14);
                }
                break;

            case 5:
                if (actionTimer.getElapsedTimeSeconds() > 0.15) {
                    outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                    setActionState(6);
                }
                break;

            case 6:
                if (!intake.isBusy() && !outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    if(intakeReady) {
                        intake.setState(IntakeConstants.INTAKE);
                        intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX - slideRangeSubtract);
                        setActionState(7);
                    }
                }
                break;

            case 7:
                if (intake.getHorizontalSlidePos() > IntakeConstants.SLIDES_MAX - slideRangeSubtract - IntakeConstants.SLIDES_ACCURACY) {
                    if (onsTimerState) {
                        actionTimer.resetTimer();
                        onsTimerState = false;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.1) {
                        intake.setState(IntakeConstants.TRANSFER);
                        setActionState(8);
                    }
                }
                break;

            case 8:
                if (!intake.isBusy()) {
                    if (onsTimerState) {
                        actionTimer.resetTimer();
                        onsTimerState = false;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.1) {
                        outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
                        setActionState(14);
                    }
                }
                break;

            case 10:
                if (actionTimer.getElapsedTimeSeconds() > 0.15) {
                    outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                    setActionState(11);
                }
                break;

            case 11:
                if (!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    setActionState(14);
                }
                break;

            case 13:
                if(!intake.isBusy() && pathState < 3){
                    intake.setState(IntakeConstants.INTAKE);
                    intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX - slideRangeSubtract - 1300);
                }

                if (!outtake.isBusy()) {
                    setActionState(-1);
                }
                break;

            case 14:
                if (!outtake.isBusy()) {
                    setActionState(-1);
                }
                break;

            case 15:
                intake.setHorizontalPosition(IntakeConstants.SLIDES_OUT + 100);
                setActionState(16);
                break;

            case 16:
                if (!intake.isBusy()) {
                    if(onsIntakeState) {
                        intake.setState(IntakeConstants.INTAKE);
                        actionTimer.resetTimer();
                        onsIntakeState = false;
                    }
                    else {
                        intake.horizontalSlidesManual(30);
                    }

                    if(actionTimer.getElapsedTimeSeconds() > 1.5)
                    {
                        intake.setState(IntakeConstants.TRANSFER);
                        setActionState(17);
                    }

                }
                break;

            case 17:
                if (!intake.isBusy()) {
                    if (onsTimerState) {
                        actionTimer.resetTimer();
                        onsTimerState = false;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.1) {
                        outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
                        setActionState(0);
                    }
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        onsScoreState = true;
        onsMoveState = true;
        pathTimer.resetTimer();
    }

    public void setActionState(int aState) {
        actionState = aState;
        onsTimerState = true;
        onsIntakeState = true;
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
        hang.update();
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
        telemetry.addData("robot in pos", robotInPos);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
