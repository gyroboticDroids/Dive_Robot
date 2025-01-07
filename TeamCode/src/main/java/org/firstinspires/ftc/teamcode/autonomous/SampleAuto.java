package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
    private Timer actionTimer;
    private Intake intake;
    private Outtake outtake;
    private int pathState;
    private int actionState;
    private Path currentPath;

    private final Pose startPose = new Pose(7.125, 102.5, Math.toRadians(0));
    private final Pose scoreSample = new Pose(13, 131, Math.toRadians(-45));
    private final Pose sampleRight = new Pose(16, 121, Math.toRadians(0));
    private final Pose sampleMiddle = new Pose(20, 131, Math.toRadians(0));
    private final Pose sampleLeft = new Pose(24, 136, Math.toRadians(15));
    private final Pose park = new Pose(60, 94, Math.toRadians(0));

    private Path scorePreload, collectSampleRight, scoreSampleRight, collectSampleMiddle, scoreSampleMiddle, collectSampleLeft, scoreSampleLeft, touchBar;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scoreSample)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scoreSample.getHeading());

        collectSampleRight = new Path(new BezierLine(new Point(scoreSample), new Point(sampleRight)));
        collectSampleRight.setLinearHeadingInterpolation(scoreSample.getHeading(), sampleRight.getHeading());

        scoreSampleRight = new Path(new BezierLine(new Point(sampleRight), new Point(scoreSample)));
        scoreSampleRight.setLinearHeadingInterpolation(sampleRight.getHeading(), scoreSample.getHeading());

        collectSampleMiddle = new Path(new BezierLine(new Point(scoreSample), new Point(sampleMiddle)));
        collectSampleMiddle.setLinearHeadingInterpolation(scoreSample.getHeading(), sampleMiddle.getHeading());

        scoreSampleMiddle = new Path(new BezierLine(new Point(sampleMiddle), new Point(scoreSample)));
        scoreSampleMiddle.setLinearHeadingInterpolation(sampleMiddle.getHeading(), scoreSample.getHeading());

        collectSampleLeft = new Path(new BezierLine(new Point(scoreSample), new Point(sampleLeft)));
        collectSampleLeft.setLinearHeadingInterpolation(scoreSample.getHeading(), sampleLeft.getHeading());

        scoreSampleLeft = new Path(new BezierLine(new Point(sampleLeft), new Point(scoreSample)));
        scoreSampleLeft.setLinearHeadingInterpolation(sampleLeft.getHeading(), scoreSample.getHeading());

        touchBar = new Path(new BezierCurve(new Point(scoreSample), new Point(park.getX(), scoreSample.getY()), new Point(park)));
        touchBar.setLinearHeadingInterpolation(scoreSample.getHeading(), park.getHeading());
    }

    public void autonomousPathUpdate() {
        boolean robotInPos = MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getX(), follower.getPose().getX(), 1) &&
                MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getY(), follower.getPose().getY(), 1);

        switch (pathState) {
            case 0:
                currentPath = scorePreload;
                follower.followPath(currentPath, true);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(robotInPos){
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                        currentPath = collectSampleRight;
                        follower.followPath(currentPath, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(robotInPos) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                        currentPath = scoreSampleRight;
                        follower.followPath(currentPath, true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(robotInPos) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                        currentPath = collectSampleMiddle;
                        follower.followPath(currentPath, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(robotInPos) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                        currentPath = scoreSampleMiddle;
                        follower.followPath(currentPath, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(robotInPos) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                        currentPath = collectSampleLeft;
                        follower.followPath(currentPath, true);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(robotInPos) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                        currentPath = scoreSampleLeft;
                        follower.followPath(currentPath, true);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(robotInPos) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                        currentPath = touchBar;
                        follower.followPath(currentPath,true);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
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
                setActionState(1);
                break;

            case 1:
                if(!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                    setActionState(-1);
                }
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
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setActionState(int aState) {
        actionState = aState;
        actionTimer.resetTimer();
    }

    @Override
    public void init()
    {
        intake = new Intake(hardwareMap);
        intake.setState(IntakeConstants.RESET_POS);

        outtake = new Outtake(hardwareMap);

        pathTimer = new Timer();
        actionTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
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
    public void loop()
    {
        follower.update();
        //outtake.update();
        //intake.update();
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
