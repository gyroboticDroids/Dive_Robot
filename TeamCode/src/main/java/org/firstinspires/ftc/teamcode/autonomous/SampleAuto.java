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
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Outtake;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "4 sample auto", group = "autonomous", preselectTeleOp = "Master Tele-op")
public class SampleAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private Intake intake;
    private Outtake outtake;
    private int pathState;
    private int actionState;
    private Path currentPath;

    private Path scorePreload, collectSampleRight, scoreSampleRight, collectSampleMiddle, scoreSampleMiddle, collectSampleLeft, scoreSampleLeft, touchBar;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_START), new Point(AutoConstants.SAMPLE_SCORE)));
        scorePreload.setLinearHeadingInterpolation(AutoConstants.SAMPLE_START.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading());

        collectSampleRight = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_SCORE), new Point(AutoConstants.SAMPLE_RIGHT)));
        collectSampleRight.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SAMPLE_RIGHT.getHeading());

        scoreSampleRight = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_RIGHT), new Point(AutoConstants.SAMPLE_SCORE)));
        scoreSampleRight.setLinearHeadingInterpolation(AutoConstants.SAMPLE_RIGHT.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading());

        collectSampleMiddle = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_SCORE), new Point(AutoConstants.SAMPLE_MIDDLE)));
        collectSampleMiddle.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SAMPLE_MIDDLE.getHeading());

        scoreSampleMiddle = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_MIDDLE), new Point(AutoConstants.SAMPLE_SCORE)));
        scoreSampleMiddle.setLinearHeadingInterpolation(AutoConstants.SAMPLE_MIDDLE.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading());

        collectSampleLeft = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_SCORE), new Point(AutoConstants.SAMPLE_LEFT)));
        collectSampleLeft.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SAMPLE_LEFT.getHeading());

        scoreSampleLeft = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_LEFT), new Point(AutoConstants.SAMPLE_SCORE)));
        scoreSampleLeft.setLinearHeadingInterpolation(AutoConstants.SAMPLE_LEFT.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading());

        touchBar = new Path(new BezierCurve(new Point(AutoConstants.SAMPLE_SCORE), new Point(AutoConstants.SAMPLE_PARK.getX(), AutoConstants.SAMPLE_SCORE.getY()), new Point(AutoConstants.SAMPLE_PARK)));
        touchBar.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SAMPLE_PARK.getHeading());
    }

    public void autonomousPathUpdate() {
        boolean robotInPos = MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getX(), follower.getPose().getX(), 1) &&
                MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getY(), follower.getPose().getY(), 1);

        switch (pathState) {
            case 0:
                currentPath = scorePreload;
                follower.followPath(currentPath, true);
                setActionState(0);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                if(robotInPos){
                    if(actionState == -1) {
                        if(outtake.getState().equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
                            setActionState(1);
                        }
                        else {
                            setActionState(10);
                            currentPath = collectSampleRight;
                            follower.followPath(currentPath, true);
                            setPathState(2);
                        }
                    }
                }
                break;
            case 2:
                if(robotInPos) {
                    if(actionState == -1) {
                        currentPath = scoreSampleRight;
                        follower.followPath(currentPath, true);
                        setActionState(0);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(robotInPos) {
                    if(actionState == -1) {
                        if(outtake.getState().equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
                            setActionState(1);
                        }
                        else {
                            currentPath = collectSampleMiddle;
                            follower.followPath(currentPath, true);
                            setActionState(10);
                            setPathState(4);
                        }
                    }
                }
                break;
            case 4:
                if(robotInPos) {
                    if(actionState == -1) {
                        currentPath = scoreSampleMiddle;
                        follower.followPath(currentPath, true);
                        setActionState(0);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(robotInPos) {
                    if(actionState == -1) {
                        if(outtake.getState().equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
                            setActionState(1);
                        }
                        else {
                            currentPath = collectSampleLeft;
                            follower.followPath(currentPath, true);
                            setActionState(10);
                            setPathState(6);
                        }
                    }
                }
                break;
            case 6:
                if(robotInPos) {
                    if(actionState == -1) {
                        currentPath = scoreSampleLeft;
                        follower.followPath(currentPath, true);
                        setActionState(0);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if(robotInPos) {
                    if(actionState == -1) {
                        if(outtake.getState().equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
                            setActionState(1);
                        }
                        else {
                            currentPath = touchBar;
                            follower.followPath(currentPath,true);
                            setActionState(20);
                            setPathState(8);
                        }
                    }
                }
                break;
            case 8:
                if(robotInPos) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
                setActionState(14);
                break;

            case 1:
                outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                setActionState(14);
                break;

            case 10:
                outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                intake.setState(IntakeConstants.INTAKE_SUB_READY);
                setActionState(11);
                break;

            case 11:
                if(!intake.isBusy()) {
                    intake.setState(IntakeConstants.INTAKE);
                    intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX);
                    setActionState(12);
                }
                break;

            case 12:
                if(intake.getHorizontalSlidePos() > IntakeConstants.SLIDES_MAX - 10) {
                    intake.setState(IntakeConstants.TRANSFER);
                    setActionState(13);
                }
                break;

            case 13:
                if(!intake.isBusy()) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
                    setActionState(14);
                }
                break;

            case 14:
                if(!outtake.isBusy()) {
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
        pathTimer.resetTimer();
    }

    public void setActionState(int aState) {
        actionState = aState;
    }

    @Override
    public void init()
    {
        intake = new Intake(hardwareMap);
        intake.setState(IntakeConstants.RESET_POS);

        outtake = new Outtake(hardwareMap);

        pathTimer = new Timer();

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
        setPathState(0);
    }

    @Override
    public void loop()
    {
        follower.update();
        outtake.update();
        intake.update();
        autonomousPathUpdate();
        autonomousActionUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
