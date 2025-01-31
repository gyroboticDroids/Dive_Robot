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

@Autonomous(name = "6 sample auto", group = "autonomous", preselectTeleOp = "Master Tele-op")
public class SixSampleAuto extends OpMode {
    private static final int OUTTAKE_UP = 500;

    private int slideRangeSubtract = 0;

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
    private boolean prevGp1Dpad = false;
    private boolean prevGp1Start = false;
    private Path currentPath;
    private double currentHeading;
    private double xSubPos = 60;
    private boolean alliancePartnerAuto = true;
    private boolean builtPaths = false;
    private boolean robotInPos;
    private boolean intakeReady = true;

    private Path scorePreload, collectSampleObs, scoreSampleObs, collectSampleRight, scoreSampleRight, collectSampleCenter, scoreSampleCenter, collectSampleLeft, scoreSampleLeft,
            collectSampleSub, scoreSampleSub, touchBar;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(AutoConstants.SIX_SAMPLE_START), new Point(AutoConstants.SAMPLE_SCORE_OBS)));
        scorePreload.setLinearHeadingInterpolation(AutoConstants.SIX_SAMPLE_START.getHeading(), AutoConstants.SAMPLE_SCORE_OBS.getHeading(), 0.6);

        if(alliancePartnerAuto) {
            collectSampleObs = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_SCORE_OBS), new Point(AutoConstants.SAMPLE_OBS)));
            collectSampleObs.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE_OBS.getHeading(), AutoConstants.SAMPLE_OBS.getHeading(), 0.8);
            collectSampleObs.setZeroPowerAccelerationMultiplier(3.5);

            scoreSampleObs = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_OBS), new Point(AutoConstants.SAMPLE_SCORE_RIGHT)));
            scoreSampleObs.setLinearHeadingInterpolation(AutoConstants.SAMPLE_OBS.getHeading(), AutoConstants.SAMPLE_SCORE_RIGHT.getHeading(), 0.6);
            scoreSampleObs.setZeroPowerAccelerationMultiplier(2);
        }
        else {
            collectSampleObs = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_SCORE_OBS), new Point(AutoConstants.SAMPLE_ALLIANCE_PARTNER)));
            collectSampleObs.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE_OBS.getHeading(), AutoConstants.SAMPLE_ALLIANCE_PARTNER.getHeading(), 0.8);
            collectSampleObs.setZeroPowerAccelerationMultiplier(3.5);

            scoreSampleObs = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_ALLIANCE_PARTNER), new Point(AutoConstants.SAMPLE_SCORE_RIGHT)));
            scoreSampleObs.setLinearHeadingInterpolation(AutoConstants.SAMPLE_ALLIANCE_PARTNER.getHeading(), AutoConstants.SAMPLE_SCORE_RIGHT.getHeading(), 0.6);
            scoreSampleObs.setZeroPowerAccelerationMultiplier(2);
        }

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

        collectSampleSub = new Path(new BezierCurve(new Point(AutoConstants.SAMPLE_SCORE_LEFT), new Point(AutoConstants.SAMPLE_SUB.getX(), AutoConstants.SAMPLE_SCORE.getY()), new Point(xSubPos, AutoConstants.SAMPLE_SUB.getY())));
        collectSampleSub.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE_LEFT.getHeading(), AutoConstants.SAMPLE_SUB.getHeading());
        collectSampleSub.setZeroPowerAccelerationMultiplier(3.5);

        scoreSampleSub = new Path(new BezierCurve(new Point(xSubPos, AutoConstants.SAMPLE_SUB.getY()), new Point(AutoConstants.SAMPLE_SUB.getX(), AutoConstants.SAMPLE_SCORE.getY()), new Point(AutoConstants.SAMPLE_SCORE)));
        scoreSampleSub.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SUB.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading());
        scoreSampleSub.setZeroPowerAccelerationMultiplier(3);

        touchBar = new Path(new BezierCurve(new Point(AutoConstants.SAMPLE_SCORE), new Point(AutoConstants.SAMPLE_PARK.getX(), AutoConstants.SAMPLE_SCORE_LEFT.getY()), new Point(AutoConstants.SAMPLE_PARK)));
        touchBar.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SAMPLE_PARK.getHeading());
        touchBar.setZeroPowerAccelerationMultiplier(3);

        builtPaths = true;
    }

    public void autonomousPathUpdate() {
        robotInPos = MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getX(), follower.getPose().getX(), 1.5) &&
                MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getY(), follower.getPose().getY(), 1.5) &&
                MathFunctions.roughlyEquals(currentHeading, follower.getPose().getHeading(), Math.toRadians(5));

        switch (pathState) {
            case 0:
                setActionState(0);
                currentHeading = currentPath.getHeadingGoal(1);
                follower.followPath(currentPath, true);
                setPathState(1);
                break;

            case 1:
                if (robotInPos) {
                    if (actionState == -1 || intake.getState().equals(IntakeConstants.TRANSFER)) {
                        if(onsScoreState) {
                            intakeReady = false;
                            setActionState(20);
                            onsScoreState = false;
                        }
                        else {
                            currentPath = scoreSampleObs;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            setPathState(2);
                        }
                    } else if (intake.getState().equals(IntakeConstants.INTAKE)) {
                        if(onsMoveState) {
                            currentPath = collectSampleObs;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            onsMoveState = false;
                        }else {
                            intakeReady = true;
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
                            currentPath = scoreSampleRight;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            setActionState(0);
                            setPathState(3);
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

            case 3:
                if (robotInPos) {
                    if (actionState == -1) {
                        if(onsScoreState) {
                            slideRangeSubtract = 100;
                            setActionState(5);
                            onsScoreState = false;
                        }
                        else {
                            currentPath = scoreSampleCenter;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            setActionState(0);
                            setPathState(4);
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

            case 4:
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
                            setPathState(5);
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

            case 5:
                if (robotInPos) {
                    if (actionState == -1) {
                        if(onsScoreState) {
                            setActionState(10);
                            onsScoreState = false;
                        }
                    }else if (actionState == 11) {
                        if(onsMoveState) {
                            currentPath = collectSampleSub;
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
                        if(onsScoreState) {
                            setActionState(15);
                            onsScoreState = false;
                        }
                    }else if (actionState == 17) {
                        if(onsMoveState) {
                            currentPath = scoreSampleSub;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            setPathState(7);
                        }
                    }
                }
                break;

            case 7:
                if (robotInPos) {
                    if (actionState == -1 || actionState == 11) {
                        if(onsScoreState){
                            setActionState(10);
                            hang.setState(HangConstants.TOUCH_BAR);
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
                if(!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);

                    if (pathState < 6) {
                        intake.setState(IntakeConstants.INTAKE_SUB_READY);
                        setActionState(13);
                    } else {
                        setActionState(14);
                    }
                }
                break;

            case 5:
                if (actionTimer.getElapsedTimeSeconds() > 0.15) {
                    outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                    setActionState(6);
                }
                break;

            case 6:
                if (outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE_READY) && intake.getHorizontalPosition() == IntakeConstants.SLIDES_MAX - slideRangeSubtract) {
                    setActionState(7);
                }

                if (!intake.isBusy()) {
                    if(intakeReady) {
                        intake.setState(IntakeConstants.INTAKE);
                        intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX - slideRangeSubtract);
                    }
                }

                if(!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                }
                break;

            case 7:
                if (intake.getHorizontalSlidePos() > IntakeConstants.SLIDES_MAX - slideRangeSubtract - IntakeConstants.SLIDES_ACCURACY) {
                    if (onsTimerState) {
                        actionTimer.resetTimer();
                        onsTimerState = false;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0) {
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
                if(!intake.isBusy() && pathState < 4 && pathState != 1){
                    intake.setState(IntakeConstants.INTAKE);
                    intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX - slideRangeSubtract - 1300);
                } else if (!intake.isBusy() && pathState == 5) {
                    intake.setHorizontalPosition(IntakeConstants.SLIDES_OUT + 100);
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
                        intake.horizontalSlidesManual(50);
                    }

                    if(actionTimer.getElapsedTimeSeconds() > 1)
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
                    if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                        outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
                        setActionState(0);
                    }
                }
                break;

            case 20:
                if (actionTimer.getElapsedTimeSeconds() > 0.15) {
                    outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                    setActionState(21);
                }
                break;

            case 21:
                if (outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE_READY) && intake.getHorizontalPosition() == IntakeConstants.SLIDES_MAX - 100) {
                    setActionState(22);
                }

                if (!intake.isBusy()) {
                    intake.setState(IntakeConstants.INTAKE);
                    intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX - 100);
                }

                if(!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                }
                break;

            case 22:
                if (intake.getHorizontalSlidePos() > IntakeConstants.SLIDES_MAX - 100 - IntakeConstants.SLIDES_ACCURACY) {
                    if (onsTimerState) {
                        actionTimer.resetTimer();
                        onsTimerState = false;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0) {
                        if(intakeReady) {
                            intake.setState(IntakeConstants.TRANSFER);
                            setActionState(23);
                        }
                    }
                }
                break;

            case 23:
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
        intake.setIntakeWheelsKeepSpinning(true);

        outtake = new Outtake(hardwareMap);
        hang = new Hang(hardwareMap);
        hang.setState(HangConstants.START);

        pathTimer = new Timer();
        actionTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(AutoConstants.SIX_SAMPLE_START);
        buildPaths();

        currentPath = scorePreload;

        intake.setState(IntakeConstants.RESET_POS);
    }

    @Override
    public void init_loop()
    {
        //Resets intake pos
        intake.update();

        if (!intake.isBusy() && intake.getState().equals(IntakeConstants.RESET_POS))
        {
            intake.setState(IntakeConstants.TRANSFER);

            telemetry.addLine("initialized!");
        }

        if(!prevGp1Dpad && gamepad1.dpad_up){
            xSubPos++;
            builtPaths = false;
        } else if (!prevGp1Dpad && gamepad1.dpad_down) {
            xSubPos--;
            builtPaths = false;
        }

        xSubPos = MathFunctions.clamp(xSubPos, 58, 80);

        if(gamepad1.dpad_left) {
            alliancePartnerAuto = false;
            builtPaths = false;
        } else if(gamepad1.dpad_right) {
            alliancePartnerAuto = true;
            builtPaths = false;
        }

        if(!prevGp1Start && gamepad1.start){
            buildPaths();
        }

        prevGp1Dpad = gamepad1.dpad_up || gamepad1.dpad_down;
        prevGp1Start = gamepad1.start;

        telemetry.addData("Sub offset (g1 dpad up and down)", xSubPos);
        telemetry.addData("Alliance partner auto (g1 dpad left and right)", alliancePartnerAuto);

        if(!builtPaths){
            telemetry.addLine("DON'T FORGET TO BUILD PATHS TO SAVE CHANGES (g1 start)");
        }
        else {
            telemetry.addLine("Paths built");
        }

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
