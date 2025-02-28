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
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.teleop.Hang;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Outtake;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "6 sample auto", group = "autonomous samples", preselectTeleOp = "Master Tele-op")
public class SixSampleAuto extends OpMode {
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
    private boolean prevGp1Bumper = false;
    private Path currentPath;
    private double currentHeading;
    private double xSubPos = 55;
    private double horiSubPos = 0;
    private boolean alliancePartnerAuto = true;
    private boolean allianceColorRed = true;
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
            collectSampleObs.setZeroPowerAccelerationMultiplier(3);

            scoreSampleObs = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_OBS), new Point(AutoConstants.SAMPLE_SCORE_RIGHT)));
            scoreSampleObs.setLinearHeadingInterpolation(AutoConstants.SAMPLE_OBS.getHeading(), AutoConstants.SAMPLE_SCORE_RIGHT.getHeading(), 0.6);
            scoreSampleObs.setZeroPowerAccelerationMultiplier(1.75);
        }
        else {
            collectSampleObs = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_SCORE_OBS), new Point(AutoConstants.SAMPLE_ALLIANCE_PARTNER)));
            collectSampleObs.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE_OBS.getHeading(), AutoConstants.SAMPLE_ALLIANCE_PARTNER.getHeading(), 0.8);
            collectSampleObs.setZeroPowerAccelerationMultiplier(3);

            scoreSampleObs = new Path(new BezierLine(new Point(AutoConstants.SAMPLE_ALLIANCE_PARTNER), new Point(AutoConstants.SAMPLE_SCORE_RIGHT)));
            scoreSampleObs.setLinearHeadingInterpolation(AutoConstants.SAMPLE_ALLIANCE_PARTNER.getHeading(), AutoConstants.SAMPLE_SCORE_RIGHT.getHeading(), 0.6);
            scoreSampleObs.setZeroPowerAccelerationMultiplier(1.75);
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
        collectSampleSub.setZeroPowerAccelerationMultiplier(2.5);

        scoreSampleSub = new Path(new BezierCurve(new Point(xSubPos, AutoConstants.SAMPLE_SUB.getY()), new Point(AutoConstants.SAMPLE_SUB.getX(), AutoConstants.SAMPLE_SCORE.getY()), new Point(AutoConstants.SAMPLE_SCORE)));
        scoreSampleSub.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SUB.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading());
        scoreSampleSub.setZeroPowerAccelerationMultiplier(2);

        touchBar = new Path(new BezierCurve(new Point(AutoConstants.SAMPLE_SCORE), new Point(AutoConstants.SAMPLE_PARK.getX(), AutoConstants.SAMPLE_SCORE_LEFT.getY()), new Point(AutoConstants.SAMPLE_PARK)));
        touchBar.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SAMPLE_PARK.getHeading());
        touchBar.setZeroPowerAccelerationMultiplier(2);

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
                            slideRangeSubtract = 0;
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
                    if (actionTimer.getElapsedTimeSeconds() > 0.25 || intake.getSampleColor() > 0) {
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
                    if (actionTimer.getElapsedTimeSeconds() > 0) {
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
                intake.setHorizontalPosition(IntakeConstants.SLIDES_OUT + (horiSubPos - 5.75) * IntakeConstants.SLIDES_TICKS_PER_INCH);
                setActionState(16);
                break;

            case 16:
                if (!intake.isBusy() && MathFunctions.roughlyEquals(intake.getHorizontalPosition(), intake.getHorizontalSlidePos(), 100)) {
                    if(onsTimerState) {
                        intake.setState(IntakeConstants.INTAKE);
                        actionTimer.resetTimer();
                        onsTimerState = false;
                    }
                    else {
                        intake.horizontalSlidesManual(20);
                    }

                    if(actionTimer.getElapsedTimeSeconds() > 3 || intake.getSampleColor() == 1 || intake.getSampleColor() == ((allianceColorRed)? 2:3))
                    {
                        if(intake.getSampleColor() == 1 || intake.getSampleColor() == ((allianceColorRed)? 2:3)) {
                            intake.setState(IntakeConstants.TRANSFER);
                            setActionState(17);
                        }
                        else {
                            intake.setState(IntakeConstants.REJECT);
                            setActionState(-100);
                        }
                    }

                }
                break;

            case 17:
                if (!intake.isBusy()) {
                    if (onsTimerState) {
                        actionTimer.resetTimer();
                        onsTimerState = false;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0) {
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
                    if ((alliancePartnerAuto)? actionTimer.getElapsedTimeSeconds() > 0.25 || intake.getSampleColor() > 0 : actionTimer.getElapsedTimeSeconds() > 0.75 || intake.getSampleColor() > 0) {
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
                    //this text stops warning
                    if (actionTimer.getElapsedTimeSeconds() > 0) {
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
        TransferConstants.resetConstants();

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

        intake.setState(IntakeConstants.START);
    }

    @Override
    public void init_loop()
    {
        //Resets intake pos
        intake.update();

        if(actionTimer.getElapsedTimeSeconds() > 4 && !(intake.getState().equals(IntakeConstants.RESET_POS) || intake.getState().equals(IntakeConstants.TRANSFER))){
            intake.setState(IntakeConstants.RESET_POS);
        }

        if (!intake.isBusy() && intake.getState().equals(IntakeConstants.RESET_POS))
        {
            intake.setState(IntakeConstants.TRANSFER);
        }

        if(!prevGp1Dpad && gamepad1.dpad_up){
            xSubPos++;
            builtPaths = false;
        } else if (!prevGp1Dpad && gamepad1.dpad_down) {
            xSubPos--;
            builtPaths = false;
        }

        xSubPos = MathFunctions.clamp(xSubPos, 55, 55 + 38);

        if(!prevGp1Bumper && gamepad1.right_bumper){
            horiSubPos += 1;
        } else if (!prevGp1Bumper && gamepad1.left_bumper) {
            horiSubPos -= 1;
        }

        horiSubPos = MathFunctions.clamp(horiSubPos, 5.75, 24);

        if(gamepad1.dpad_left) {
            alliancePartnerAuto = false;
            builtPaths = false;
        } else if(gamepad1.dpad_right) {
            alliancePartnerAuto = true;
            builtPaths = false;
        }

        if(gamepad1.b) {
            allianceColorRed = true;
        } else if(gamepad1.x) {
            allianceColorRed = false;
        }

        if(!prevGp1Start && gamepad1.start){
            buildPaths();
        }

        prevGp1Dpad = gamepad1.dpad_up || gamepad1.dpad_down;
        prevGp1Bumper = gamepad1.left_bumper || gamepad1.right_bumper;
        prevGp1Start = gamepad1.start;

        telemetry.addData("Sub drive offset (g1 dpad up and down) [22 is middle]", xSubPos - 49);
        telemetry.addData("Sub intake offset (g1 bumper right and left) [13.5 is middle]", horiSubPos);
        telemetry.addData("Alliance partner auto (g1 dpad left and right)", alliancePartnerAuto);

        telemetry.addData("Is alliance color red (g1 x and b)", allianceColorRed);

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
        TransferConstants.isAllianceRed = allianceColorRed;
        setPathState(0);
    }

    @Override
    public void stop() {
        TransferConstants.horiSlidePos = intake.getHorizontalSlidePos();
        TransferConstants.heading = Math.toDegrees(follower.getPose().getHeading());
        TransferConstants.endPose = follower.getPose();
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
        telemetry.addData("intake sample color", intake.getSampleColor());
        telemetry.addData("robot in pos", robotInPos);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
