package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Outtake;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Disabled
@Autonomous(name = "5 + 1 / 5 + 0 no preload", group = "autonomous specimens", preselectTeleOp = "Master Tele-op")
public class FiveSpecimenOneSampleAutoPushing extends OpMode {
    private static final double SLOW_ZERO_POWER_ACCEL = 1.5;
    private static final double COLLECT_ZERO_POWER_ACCEL = 1.6;

    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;
    private Timer runTimer;
    private Intake intake;
    private Outtake outtake;
    private int pathState = -1, actionState = -1;
    private int loopCount;
    private boolean ons = false;
    private boolean isScoreSample = true;

    private Path scorePreload, grabSpecimen1, unjamSample, scoreSpecimen1,
            scoreSample, park, pushing, startPushing, pushing0, pushing1, pushing2, pushing3, pushing4;

    private PathChain grabSpecimenReady, scoreSpecimen;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_START), new Point(AutoConstants.SPECIMEN_SCORE_PRELOAD_OLD)));
        scorePreload.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_START.getHeading(), AutoConstants.SPECIMEN_SCORE_PRELOAD_OLD.getHeading());
        scorePreload.setZeroPowerAccelerationMultiplier(4);

        startPushing = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_START), new Point(AutoConstants.SPECIMEN_START_NO_PRELOAD), AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT2, new Point(AutoConstants.SPECIMEN_PUSHING2)));
        startPushing.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_START.getHeading(), AutoConstants.SPECIMEN_PUSHING2.getHeading());
        startPushing.setZeroPowerAccelerationMultiplier(2.5);

        pushing = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_SCORE_PRELOAD_OLD), AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT0, AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT1, new Point(AutoConstants.SPECIMEN_PUSHING2)));
        pushing.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_START_NO_PRELOAD.getHeading(), AutoConstants.SPECIMEN_PUSHING2.getHeading());
        pushing.setZeroPowerAccelerationMultiplier(3);

        pushing0 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_PUSHING2), new Point(AutoConstants.SPECIMEN_PUSHING3)));
        pushing0.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING2.getHeading(), AutoConstants.SPECIMEN_PUSHING3.getHeading());
        pushing0.setZeroPowerAccelerationMultiplier(4.5);

        pushing1 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_PUSHING3), AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT4, new Point(AutoConstants.SPECIMEN_PUSHING4)));
        pushing1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING3.getHeading(), AutoConstants.SPECIMEN_PUSHING4.getHeading());
        pushing1.setZeroPowerAccelerationMultiplier(3.5);

        pushing2 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_PUSHING4), new Point(AutoConstants.SPECIMEN_PUSHING5)));
        pushing2.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING4.getHeading(), AutoConstants.SPECIMEN_PUSHING5.getHeading());
        pushing2.setZeroPowerAccelerationMultiplier(4.5);

        pushing3 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_PUSHING5), AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT6 ,new Point(AutoConstants.SPECIMEN_PUSHING6)));
        pushing3.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING5.getHeading(), AutoConstants.SPECIMEN_PUSHING6.getHeading());
        pushing3.setZeroPowerAccelerationMultiplier(2.5);

        pushing4 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_PUSHING6), AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT7, new Point(AutoConstants.SPECIMEN_PUSHING7)));
        pushing4.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING6.getHeading(), AutoConstants.SPECIMEN_PUSHING7.getHeading());
        pushing4.setZeroPowerAccelerationMultiplier(2.5);

        grabSpecimen1 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_PUSHING7), new Point(AutoConstants.SPECIMEN_GRAB_AFTER_PUSHING)));
        grabSpecimen1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING7.getHeading(), AutoConstants.SPECIMEN_GRAB_AFTER_PUSHING.getHeading());
        grabSpecimen1.setZeroPowerAccelerationMultiplier(COLLECT_ZERO_POWER_ACCEL);

        unjamSample = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB_AFTER_PUSHING), AutoConstants.SPECIMEN_UNJAM_POINT, new Point(AutoConstants.SPECIMEN_GRAB)));
        unjamSample.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB_AFTER_PUSHING.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading());
        unjamSample.setZeroPowerAccelerationMultiplier(0.8);

        scoreSpecimen1 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB_AFTER_PUSHING), AutoConstants.SPECIMEN_CONTROL_POINT, new Point(AutoConstants.SPECIMEN_SCORE)));
        scoreSpecimen1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB_AFTER_PUSHING.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
        scoreSpecimen1.setZeroPowerAccelerationMultiplier(SLOW_ZERO_POWER_ACCEL);

        grabSpecimenReady = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(AutoConstants.SPECIMEN_SCORE), AutoConstants.SPECIMEN_SCORING_CONTROL_POINT1, AutoConstants.SPECIMEN_SCORING_CONTROL_POINT2, new Point(AutoConstants.SPECIMEN_GRAB)))
                .setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading())
                .setZeroPowerAccelerationMultiplier(COLLECT_ZERO_POWER_ACCEL)
                .build();

        scoreSpecimen = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB), AutoConstants.SPECIMEN_SCORING_CONTROL_POINT3, new Point(AutoConstants.SPECIMEN_SCORE)))
                .setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading())
                .setZeroPowerAccelerationMultiplier(SLOW_ZERO_POWER_ACCEL)
                .build();

        scoreSample = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SAMPLE_SCORE.getX(), AutoConstants.SPECIMEN_GRAB.getY() + 20), new Point(AutoConstants.SAMPLE_SCORE)));
        scoreSample.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading(), 0.5);
        scoreSample.setZeroPowerAccelerationMultiplier(4);

        park = new Path(new BezierCurve(new Point(AutoConstants.SAMPLE_SCORE), new Point(AutoConstants.SPECIMEN_PARK)));
        park.setLinearHeadingInterpolation(AutoConstants.SAMPLE_SCORE.getHeading(), AutoConstants.SPECIMEN_PARK.getHeading(), 0.5);
        park.setZeroPowerAccelerationMultiplier(4);
    }

    public void autonomousPathUpdate() {
        boolean robotInPos = follower.getCurrentTValue() >= 0.985;

        switch (pathState) {
            case 100:
                follower.followPath(startPushing);
                setActionState(12);
                setPathState(1);
                break;

            case 0:
                follower.followPath(scorePreload);
                setActionState(10);
                setPathState(1);
                break;

            case 1:
                if(robotInPos && actionState == -1) {
                    follower.followPath(pushing);
                    setActionState(11);
                    setPathState(2);
                }
                break;

            case 2:
                if(robotInPos) {
                    follower.followPath(pushing0);
                    intake.setState(IntakeConstants.INTAKE_SUB_READY);
                    setPathState(3);
                }
                break;

            case 3:
                if(robotInPos) {
                    follower.followPath(pushing1);
                    setPathState(4);
                }
                break;

            case 4:
                if(robotInPos) {
                    follower.followPath(pushing2);
                    setPathState(5);
                }
                break;

            case 5:
                if(robotInPos) {
                    follower.followPath(pushing3);
                    setPathState(6);
                }
                break;

            case 6:
                if(robotInPos) {
                    follower.followPath(pushing4);
                    intake.setState(IntakeConstants.START);
                    setPathState(7);
                }
                break;

            case 7:
                if(robotInPos) {
                    if(actionState == -1) {
                        follower.followPath(grabSpecimen1);
                        setPathState(8);
                    }
                }
                break;

            case 8:
                if(robotInPos || !ons) {
                    if(actionState == -1 || pathTimer.getElapsedTimeSeconds() > 0.2) {
                        if(ons){
                            setActionState(0);
                            ons = false;
                            pathTimer.resetTimer();
                        }
                        else {
                            follower.followPath(scoreSpecimen1);
                            setPathState(9);
                        }
                    }
                } else if(pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(20);
                }
                break;

            case 20:
                //if jammed run this code
                follower.followPath(unjamSample);
                setPathState(9);
                break;

            case 9:
                if(robotInPos || pathTimer.getElapsedTimeSeconds() > 3.5) {
                    if(actionState == -1) {
                        if (ons) {
                            setActionState(1);
                            ons = false;
                        }
                        else {
                            follower.followPath(grabSpecimenReady);
                            setActionState(13);
                            setPathState(10);
                        }
                    }
                }
                break;

            case 10:
                if(robotInPos || !ons) {
                    if(actionState == -1 || pathTimer.getElapsedTimeSeconds() > 0.2) {
                        if(ons){
                            setActionState(0);
                            ons = false;
                            pathTimer.resetTimer();
                        }
                        else {
                            follower.followPath(scoreSpecimen);

                            loopCount--;
                            if(loopCount > 0) {
                                setPathState(9);
                            } else {
                                setPathState(15);
                            }
                        }
                    }
                }
                break;

            case 15:
                if(robotInPos) {
                    if(actionState == -1) {
                        if (ons) {
                            setActionState(1);
                            ons = false;
                        }
                        else {
                            follower.followPath(grabSpecimenReady);

                            if(isScoreSample) {
                                setActionState(13);
                                setPathState(16);
                            } else {
                                setActionState(2);
                                setPathState(-1);
                            }
                        }
                    }
                }
                break;

            case 16:
                if(robotInPos || !ons) {
                    if(actionState == -1) {
                        if (ons) {
                            setActionState(5);
                            ons = false;
                        }
                        else {
                            follower.followPath(scoreSample);
                            setActionState(7);
                            setPathState(17);
                        }
                    }
                }
                break;

            case 17:
                if(follower.getPose().getY() > AutoConstants.SAMPLE_SCORE.getY() - 6) {
                    if(actionState == -1) {
                        if (ons) {
                            setActionState(8);
                            ons = false;
                        }
                        else {
                            follower.followPath(park);
                            setActionState(2);
                            setPathState(-1);
                        }
                    }
                } else if(runTimer.getElapsedTimeSeconds() > 29.75) {
                    setActionState(8);
                    setPathState(-1);
                }
                break;
        }
        telemetry.addData("robot in pos", robotInPos);
    }

    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                if(actionTimer.getElapsedTimeSeconds() > 0) {
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
                    setActionState(14);
                }
                break;

            case 1:
                if(actionTimer.getElapsedTimeSeconds() > 0) {
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
                    setActionState(14);
                }
                break;

            case 2:
                if(actionTimer.getElapsedTimeSeconds() > 0.1 && follower.getPose().getX() < 35) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    setActionState(14);
                }
                break;

            case 5:
                outtake.setState(OuttakeConstants.GRAB_SAMPLE_OFF_WALL);
                setActionState(14);
                break;

            case 7:
                if(follower.getPose().getY() > 50) {
                    outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
                    setActionState(14);
                }
                break;

            case 8:
                outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                setActionState(14);
                break;


            case 10:
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_PRELOAD_READY);
                setActionState(14);
                break;

            case 11:
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_PRELOAD);
                setActionState(12);
                break;

            case 12:
                if(!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
                    setActionState(14);
                }
                break;

            case 13:
                if(follower.getPose().getX() < 35) {
                    outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
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
        ons = true;
        pathTimer.resetTimer();
    }

    public void setActionState(int aState) {
        actionState = aState;
        actionTimer.resetTimer();
    }

    @Override
    public void init()
    {
        TransferConstants.resetConstants();

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        pathTimer = new Timer();
        actionTimer = new Timer();
        runTimer = new Timer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(AutoConstants.SPECIMEN_START);
        buildPaths();

        intake.setState(IntakeConstants.START);
        outtake.setState(OuttakeConstants.SPEC_PRELOAD_START);
    }

    boolean init = false;

    @Override
    public void init_loop()
    {
        //Resets intake pos
        intake.update();
        outtake.update();

        if(actionTimer.getElapsedTimeSeconds() > 4 && !(intake.getState().equals(IntakeConstants.RESET_POS) || intake.getState().equals(IntakeConstants.TRANSFER))){
            intake.setState(IntakeConstants.RESET_POS);
        }

        if (!intake.isBusy() && intake.getState().equals(IntakeConstants.RESET_POS))
        {
            intake.setState(IntakeConstants.TRANSFER);
            init = true;
        }

        if(gamepad1.dpad_left) {
            isScoreSample = true;
        } else if(gamepad1.dpad_right) {
            isScoreSample = false;
        }

        telemetry.addData("run with preload (g1 dpad left + right)", isScoreSample);

        if(init) {
            telemetry.addLine("initialized");
        } else {
            telemetry.addLine("NOT INITIALIZED");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        runTimer.resetTimer();
        intake.setState(IntakeConstants.START);

        if(isScoreSample) {
            loopCount = 3;
            setPathState(0);
        } else {
            loopCount = 4;
            setPathState(100);
        }
    }

    @Override
    public void stop() {
        TransferConstants.horiSlidePos = intake.getHorizontalSlidePos();
        TransferConstants.heading = Math.toDegrees(follower.getPose().getHeading());
        TransferConstants.endPose = follower.getPose();
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
        telemetry.addData("parametric end", follower.atParametricEnd());
        telemetry.addData("t value", follower.getCurrentTValue());
        telemetry.addData("path state", pathState);
        telemetry.addData("action state", actionState);
        telemetry.addData("intake state", intake.getState());
        telemetry.addData("outtake state", outtake.getState());
        telemetry.addData("hori slide pos", intake.getHorizontalSlidePos());
        telemetry.addData("hori slide setpoint", intake.getHorizontalPosition());
        telemetry.addData("intake is busy", intake.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
