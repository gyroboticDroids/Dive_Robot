package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
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

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "0 + 6", group = "autonomous samples", preselectTeleOp = "Master Tele-op")
public class SixSample extends OpMode {
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
    private boolean onsSetState;
    private boolean prevGp1Dpad = false;
    private boolean prevGp2Dpad = false;
    private int allianceColor = 2;
    private double xSubPos1 = 55;
    private double xSubPos2 = 55;
    private double horiSubPos1 = 0;
    private double horiSubPos2 = 0;
    private boolean clear1 = false;
    private boolean clear2 = false;
    private boolean missed = false;

    //Bulk reading
    List<LynxModule> allHubs;

    private Path scorePreload, collectSampleRight, scoreSampleRight, collectSampleCenter, scoreSampleCenter, collectSampleLeft, scoreSampleLeft,
            collectSampleSub1, scoreSampleSub1, collectSampleSub2, scoreSampleSub2, toNext, touchBar, turn, touch;

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

        collectSampleSub1 = new Path(new BezierCurve(new Point(AutoConstants.SAMPLE_SCORE_LEFT), AutoConstants.SAMPLE_COLLECT_CONTROL, new Point(xSubPos1, AutoConstants.SAMPLE_SUB.getY())));
        collectSampleSub1.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE_LEFT.getHeading(), AutoConstants.SAMPLE_SUB.getHeading());
        collectSampleSub1.setZeroPowerAccelerationMultiplier(2);

        scoreSampleSub1 = new Path(new BezierCurve(new Point(xSubPos1, AutoConstants.SAMPLE_SUB.getY()), AutoConstants.SAMPLE_COLLECT_CONTROL, new Point(AutoConstants.SAMPLE_SCORE)));
        scoreSampleSub1.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SUB.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading());
        scoreSampleSub1.setZeroPowerAccelerationMultiplier(2);

        toNext = new Path(new BezierCurve(new Point(xSubPos1, AutoConstants.SAMPLE_SUB.getY()), new Point((xSubPos1 + xSubPos2) / 2, AutoConstants.SAMPLE_SUB.getY() + 5),
                new Point(xSubPos2, AutoConstants.SAMPLE_SUB.getY())));
        toNext.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SUB.getHeading(), AutoConstants.SAMPLE_SUB.getHeading());
        toNext.setZeroPowerAccelerationMultiplier(2);

        collectSampleSub2 = new Path(new BezierCurve(new Point(AutoConstants.SAMPLE_SCORE), AutoConstants.SAMPLE_COLLECT_CONTROL, new Point(xSubPos2, AutoConstants.SAMPLE_SUB.getY())));
        collectSampleSub2.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SAMPLE_SUB.getHeading());
        collectSampleSub2.setZeroPowerAccelerationMultiplier(2);

        scoreSampleSub2 = new Path(new BezierCurve(new Point(xSubPos2, AutoConstants.SAMPLE_SUB.getY()), AutoConstants.SAMPLE_COLLECT_CONTROL, new Point(AutoConstants.SAMPLE_SCORE)));
        scoreSampleSub2.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SUB.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading());
        scoreSampleSub2.setZeroPowerAccelerationMultiplier(2);

        turn = new Path(new BezierLine(new Point(xSubPos2, AutoConstants.SAMPLE_SUB.getY()), new Point(xSubPos2, AutoConstants.SAMPLE_SUB.getY() + 20)));
        turn.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SUB.getHeading(), AutoConstants.SAMPLE_SUB.getHeading());
        turn.setZeroPowerAccelerationMultiplier(4);

        touch = new Path(new BezierLine(new Point(xSubPos2, AutoConstants.SAMPLE_SUB.getY() + 20), new Point(AutoConstants.SAMPLE_PARK)));
        touch.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SUB.getHeading(), AutoConstants.SAMPLE_PARK.getHeading(), 0.8);
        touch.setZeroPowerAccelerationMultiplier(3);

        touchBar = new Path(new BezierCurve(new Point(AutoConstants.SAMPLE_SCORE), AutoConstants.SAMPLE_COLLECT_CONTROL, new Point(AutoConstants.SAMPLE_PARK)));
        touchBar.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SAMPLE_PARK.getHeading());
        touchBar.setZeroPowerAccelerationMultiplier(2);
    }

    public void autonomousPathUpdate() {
        boolean robotInPos = follower.getCurrentTValue() >= 0.97;
        switch (pathState) {
            case 0:
                setActionState(0);
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (robotInPos) {
                    if (actionState == -1) {
                        if (onsSetState) {
                            slideRangeSubtract = 0;
                            setActionState(5);
                            follower.followPath(collectSampleRight);
                            onsSetState = false;
                        } else {
                            follower.followPath(scoreSampleRight);
                            setActionState(0);
                            setPathState(2);
                        }
                    }
                }
                break;

            case 2:
                if (robotInPos) {
                    if (actionState == -1) {
                        if (onsSetState) {
                            slideRangeSubtract = 0;
                            setActionState(5);
                            follower.followPath(collectSampleCenter);
                            onsSetState = false;
                        } else {
                            follower.followPath(scoreSampleCenter);
                            setActionState(0);
                            setPathState(3);
                        }
                    }
                }
                break;

            case 3:
                if (robotInPos) {
                    if (actionState == -1) {
                        if (onsSetState) {
                            slideRangeSubtract = 0;
                            setActionState(5);
                            follower.followPath(collectSampleLeft);
                            onsSetState = false;
                        } else {
                            follower.followPath(scoreSampleLeft);
                            setActionState(0);
                            setPathState(4);
                        }
                    }
                }
                break;

            case 4:
                if (robotInPos) {
                    if (actionState == -1) {
                        if(onsSetState) {
                            setActionState(10);
                            onsSetState = false;
                        }
                    }else if (actionState == 11) {
                        follower.followPath(collectSampleSub1);
                        setPathState(5);
                    }
                }
                break;

            case 5:
                if (robotInPos) {
                    if (actionState == -1) {
                        if(onsSetState) {
                            setActionState(15);
                            onsSetState = false;
                        }
                    }else if (actionState == 17 && actionTimer.getElapsedTimeSeconds() > 0.5) {
                        if(!missed) {
                            follower.followPath(scoreSampleSub1);
                            setPathState(6);
                        } else {
                            if(xSubPos1 != xSubPos2) {
                                follower.followPath(toNext);
                            }
                            setPathState(200);
                        }
                    }
                }
                break;

            case 6:
                if (robotInPos) {
                    if (actionState == -1) {
                        if(onsSetState) {
                            setActionState(10);
                            onsSetState = false;
                        }
                    }else if (actionState == 11) {
                        follower.followPath(collectSampleSub2);
                        setPathState(7);
                    }
                }
                break;

            case 200:
                if (actionState == -1) {
                    setPathState(7);
                }
                break;

            case 7:
                if (robotInPos) {
                    if (actionState == -1) {
                        if(onsSetState) {
                            hang.setState(HangConstants.TOUCH_BAR);
                            setActionState(15);
                            onsSetState = false;
                        }
                    }else if (actionState == 17 && !intake.isBusy()) {
                        if(!missed) {
                            follower.followPath(scoreSampleSub2);
                            setPathState(8);
                        } else {
                            follower.followPath(turn);
                            setPathState(9);
                        }
                    }
                }
                break;

            case 8:
                if (robotInPos) {
                    if (actionState == -1 || actionState == 11) {
                        if(onsSetState){
                            setActionState(10);
                            onsSetState = false;
                        }else {
                            follower.followPath(touchBar);
                            setPathState(10);
                        }
                    }
                }
                break;

            case 9:
                if (robotInPos) {
                    follower.followPath(touch);
                    setPathState(10);
                }
                break;

            case 10:
                if (robotInPos) {
                    setPathState(-1);
                }
                break;
        }

        telemetry.addData("robot in pos", robotInPos);
    }

    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                if(!outtake.isBusy() && MathFunctions.distance(follower.getPose(), AutoConstants.SAMPLE_SCORE) < 50) {
                    if(outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE) || outtake.getState().equals(OuttakeConstants.START)) {
                        outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
                    }

                    if (pathState < 8) {
                        intake.setState(IntakeConstants.INTAKE_SUB_READY);
                        setActionState(13);
                    } else {
                        setActionState(14);
                    }
                }
                break;

            case 5:
                if (actionTimer.getElapsedTimeSeconds() > 0) {
                    outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                    setActionState(6);
                }
                break;

            case 6:
                if (outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE_READY) && intake.getHorizontalPosition() == IntakeConstants.SLIDES_MAX - slideRangeSubtract) {
                    setActionState(7);
                }

                if (!intake.isBusy()) {
                    if(onsTimerState) {
                        actionTimer.resetTimer();
                        onsTimerState = false;
                    }

                    if(pathState == 3) {
                        if(actionTimer.getElapsedTimeSeconds() > 0.25) {
                            intake.setState(IntakeConstants.INTAKE);
                            intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX - slideRangeSubtract);
                        }
                    } else {
                        intake.setState(IntakeConstants.INTAKE);
                        intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX - slideRangeSubtract);
                    }
                }

                if(!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                }
                break;

            case 7:
                if (intake.getHorizontalSlidePos() > IntakeConstants.SLIDES_MAX - slideRangeSubtract - IntakeConstants.SLIDES_ACCURACY && onsTimerState) {
                    actionTimer.resetTimer();
                    onsTimerState = false;
                }

                if ((intake.getHorizontalSlidePos() > IntakeConstants.SLIDES_MAX - slideRangeSubtract -
                        IntakeConstants.SLIDES_ACCURACY && actionTimer.getElapsedTimeSeconds() > 0.75) || intake.getSampleColor() > 0) {
                    intake.setState(IntakeConstants.TRANSFER_FAST);
                    setActionState(8);
                }
                break;

            case 8:
                if (!intake.isBusy()) {
                    if (onsTimerState) {
                        actionTimer.resetTimer();
                        onsTimerState = false;
                    }

                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
                    setActionState(14);
                }
                break;

            case 10:
                if (actionTimer.getElapsedTimeSeconds() > 0.1) {
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
                    intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX - slideRangeSubtract - 1000);
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
                intake.setHorizontalPosition(IntakeConstants.SLIDES_OUT + ((pathState < 7) ? horiSubPos1 - 5.75 : horiSubPos2 - 5.75)
                        * IntakeConstants.SLIDES_TICKS_PER_INCH);
                missed = false;
                setActionState(16);
                break;

            case 16:
                if (!intake.isBusy() && MathFunctions.roughlyEquals(intake.getHorizontalPosition(), intake.getHorizontalSlidePos(), 100)) {
                    if(onsTimerState) {
                        if((pathState < 7) ? clear1 : clear2) {
                            intake.setState(IntakeConstants.CLEAR_SUB);
                        } else {
                            intake.setState(IntakeConstants.INTAKE);
                        }
                        actionTimer.resetTimer();
                        onsTimerState = false;
                    }
                    else if(actionTimer.getElapsedTimeSeconds() > 0.5) {
                        if(!intake.getState().equals(IntakeConstants.INTAKE)) {
                            intake.setState(IntakeConstants.INTAKE);
                        }
                        intake.horizontalSlidesManual(10);
                    }

                    if(actionTimer.getElapsedTimeSeconds() > 3 || intake.getSampleColor() == 1 || intake.getSampleColor() == allianceColor)
                    {
                        if(intake.getSampleColor() == 1 || intake.getSampleColor() == allianceColor) {
                            intake.setState(IntakeConstants.TRANSFER);
                            setActionState(17);
                        }
                        else {
                            intake.setState(IntakeConstants.TRANSFER_REJECT);
                            missed = true;
                            setActionState(17);
                        }
                    }

                }
                break;

            case 17:
                if (!intake.isBusy()) {
                    if(!missed) {
                        outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
                        setActionState(0);
                    } else {
                        if(pathState < 7 || pathState == 200) {
                            intake.setState(IntakeConstants.INTAKE_SUB_READY);
                        }
                        setActionState(-1);
                    }
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        onsSetState = true;
        pathTimer.resetTimer();
    }

    public void setActionState(int aState) {
        actionState = aState;
        onsTimerState = true;
        actionTimer.resetTimer();
    }

    @Override
    public void init() {
        TransferConstants.resetConstants();

        intake = new Intake(hardwareMap);
        intake.setIntakeWheelsKeepSpinning(true);

        outtake = new Outtake(hardwareMap);
        hang = new Hang(hardwareMap, outtake);
        hang.setState(HangConstants.START);

        pathTimer = new Timer();
        actionTimer = new Timer();
        actionTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(AutoConstants.SAMPLE_START);
        buildPaths();

        intake.setState(IntakeConstants.START);
        outtake.setState(OuttakeConstants.START);

        //Bulk reading
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void init_loop()
    {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        //Resets intake pos
        intake.update();
        outtake.update();

        if(actionTimer.getElapsedTimeSeconds() > 4 && !(intake.getState().equals(IntakeConstants.RESET_POS) || intake.getState().equals(IntakeConstants.TRANSFER))){
            intake.setState(IntakeConstants.RESET_POS);
        }

        if (!intake.isBusy() && intake.getState().equals(IntakeConstants.RESET_POS))
        {
            intake.setState(IntakeConstants.TRANSFER);
        }

        if(!prevGp1Dpad && gamepad1.dpad_up){
            xSubPos1++;
        } else if (!prevGp1Dpad && gamepad1.dpad_down) {
            xSubPos1--;
        }

        if(!prevGp2Dpad && gamepad2.dpad_up){
            xSubPos2++;
        } else if (!prevGp2Dpad && gamepad2.dpad_down) {
            xSubPos2--;
        }

        xSubPos1 = MathFunctions.clamp(xSubPos1, 54, 54 + 38);
        xSubPos2 = MathFunctions.clamp(xSubPos2, 54, 54 + 38);

        if(!prevGp1Dpad && gamepad1.dpad_right){
            horiSubPos1 += 1;
        } else if (!prevGp1Dpad && gamepad1.dpad_left) {
            horiSubPos1 -= 1;
        }

        if(!prevGp2Dpad && gamepad2.dpad_right){
            horiSubPos2 += 1;
        } else if (!prevGp2Dpad && gamepad2.dpad_left) {
            horiSubPos2 -= 1;
        }

        horiSubPos1 = MathFunctions.clamp(horiSubPos1, 5.75, 24);
        horiSubPos2 = MathFunctions.clamp(horiSubPos2, 5.75, 24);

        if(gamepad1.b) {
            allianceColor = 2;
        } else if(gamepad1.x) {
            allianceColor = 3;
        }

        if(gamepad1.a) {
            clear1 = true;
        } else if(gamepad1.y) {
            clear1 = false;
        }

        if(gamepad2.a) {
            clear2 = true;
        } else if(gamepad2.y) {
            clear2 = false;
        }

        prevGp1Dpad = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;
        prevGp2Dpad = gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right;

        telemetry.addData("Sub drive offset 1 (g1 dpad up and down) [22 is middle]", xSubPos1 - 48);
        telemetry.addData("Sub intake offset 1 (g1 bumper right and left) [13.5 is middle]", horiSubPos1);
        telemetry.addData("Clear sub 1 (g1 a and y)", clear1);
        telemetry.addData("Sub drive offset 2 (g2 dpad up and down) [22 is middle]", xSubPos2 - 48);
        telemetry.addData("Sub intake offset 2 (g2 bumper right and left) [13.5 is middle]", horiSubPos2);
        telemetry.addData("Clear sub 2 (g2 a and y)", clear2);
        telemetry.addData("Alliance color (g1 x and b)", (allianceColor == 2)? "red":"blue");

        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        buildPaths();
        setPathState(0);
    }

    @Override
    public void stop() {
        TransferConstants.horiSlidePos = intake.getHorizontalSlidePos();
        TransferConstants.heading = Math.toDegrees(follower.getPose().getHeading());
        TransferConstants.endPose = follower.getPose();
        TransferConstants.allianceColor = allianceColor;
    }

    private double maxTime = 0;
    private double prevTime = 0;

    private boolean framerate = false;

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        outtake.update();
        intake.update();
        hang.update();
        follower.update();

        autonomousPathUpdate();
        autonomousActionUpdate();

        double currentTime = time - prevTime;
        prevTime = time;

        if(currentTime > maxTime && framerate) {
            maxTime = currentTime;
        }

        if(!framerate) framerate = true;

        telemetry.addLine("-------------------FPS-----------------------");
        telemetry.addData("loop speed", currentTime * 1000);
        telemetry.addData("min speed", maxTime * 1000);

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("action state", actionState);
        telemetry.addData("intake state", intake.getState());
        telemetry.addData("hori slide pos", intake.getHorizontalSlidePos());
        telemetry.addData("hori slide setpoint", intake.getHorizontalPosition());
        telemetry.addData("intake is busy", intake.isBusy());
        telemetry.addData("At parametric end", follower.atParametricEnd());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
