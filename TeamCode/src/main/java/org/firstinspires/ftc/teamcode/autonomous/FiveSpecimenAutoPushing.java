package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Outtake;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "5 specimen auto pushing", group = "autonomous specimens", preselectTeleOp = "Master Tele-op")
public class FiveSpecimenAutoPushing extends OpMode {
    private static final double SLOW_ZERO_POWER_ACCEL = 1.1;

    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;
    private Intake intake;
    private Outtake outtake;
    private int pathState = -1;
    private int actionState = -1;
    private boolean ons = false;
    private Path currentPath;

    private Path scorePreload, grabSpecimen1, scoreSpecimen1,
            grabSpecimenReady2, scoreSpecimen2, grabSpecimenReady3, scoreSpecimen3, grabSpecimenReady4, scoreSpecimen4, park, pushing, pushing0, pushing1, pushing2, pushing3, pushing4;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_START), new Point(AutoConstants.SPECIMEN_SCORE)));
        scorePreload.setLinearHeadingInterpolation(0,0);
        scorePreload.setZeroPowerAccelerationMultiplier(2);

        pushing = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_SCORE), AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT1, AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT2, new Point(AutoConstants.SPECIMEN_PUSHING2)));
        pushing.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_PUSHING2.getHeading());
        pushing.setZeroPowerAccelerationMultiplier(2.5);

        pushing0 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_PUSHING2), new Point(AutoConstants.SPECIMEN_PUSHING3)));
        pushing0.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING2.getHeading(), AutoConstants.SPECIMEN_PUSHING3.getHeading());
        pushing0.setZeroPowerAccelerationMultiplier(4);

        pushing1 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_PUSHING3), AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT4, new Point(AutoConstants.SPECIMEN_PUSHING4)));
        pushing1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING3.getHeading(), AutoConstants.SPECIMEN_PUSHING4.getHeading());
        pushing1.setZeroPowerAccelerationMultiplier(3);

        pushing2 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_PUSHING4), new Point(AutoConstants.SPECIMEN_PUSHING5)));
        pushing2.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING4.getHeading(), AutoConstants.SPECIMEN_PUSHING5.getHeading());
        pushing2.setZeroPowerAccelerationMultiplier(4);

        pushing3 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_PUSHING5), AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT6 ,new Point(AutoConstants.SPECIMEN_PUSHING6)));
        pushing3.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING5.getHeading(), AutoConstants.SPECIMEN_PUSHING6.getHeading());
        pushing3.setZeroPowerAccelerationMultiplier(3);

        pushing4 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_PUSHING6), AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT7, new Point(AutoConstants.SPECIMEN_PUSHING7)));
        pushing4.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING6.getHeading(), AutoConstants.SPECIMEN_PUSHING7.getHeading());
        pushing4.setZeroPowerAccelerationMultiplier(2.5);

        grabSpecimen1 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_PUSHING7), new Point(AutoConstants.SPECIMEN_GRAB_AFTER_PUSHING)));
        grabSpecimen1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING7.getHeading(), AutoConstants.SPECIMEN_GRAB_AFTER_PUSHING.getHeading());
        grabSpecimen1.setZeroPowerAccelerationMultiplier(1.5);

        scoreSpecimen1 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB_AFTER_PUSHING), AutoConstants.SPECIMEN_CONTROL_POINT, new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 1)));
        scoreSpecimen1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB_AFTER_PUSHING.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
        scoreSpecimen1.setZeroPowerAccelerationMultiplier(1.2);

        grabSpecimenReady2 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 1), AutoConstants.SPECIMEN_SCORING_CONTROL_POINT1,
                AutoConstants.SPECIMEN_SCORING_CONTROL_POINT2, new Point(AutoConstants.SPECIMEN_GRAB)));
        grabSpecimenReady2.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading());
        grabSpecimenReady2.setZeroPowerAccelerationMultiplier(1.2);

        scoreSpecimen2 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 2)));
        scoreSpecimen2.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
        scoreSpecimen2.setZeroPowerAccelerationMultiplier(SLOW_ZERO_POWER_ACCEL);

        grabSpecimenReady3 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 2), AutoConstants.SPECIMEN_SCORING_CONTROL_POINT1,
                AutoConstants.SPECIMEN_SCORING_CONTROL_POINT2, new Point(AutoConstants.SPECIMEN_GRAB)));
        grabSpecimenReady3.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading());
        grabSpecimenReady3.setZeroPowerAccelerationMultiplier(1.2);

        scoreSpecimen3 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 3)));
        scoreSpecimen3.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
        scoreSpecimen3.setZeroPowerAccelerationMultiplier(SLOW_ZERO_POWER_ACCEL);

        grabSpecimenReady4 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 3), AutoConstants.SPECIMEN_SCORING_CONTROL_POINT1,
                AutoConstants.SPECIMEN_SCORING_CONTROL_POINT2, new Point(AutoConstants.SPECIMEN_GRAB)));
        grabSpecimenReady4.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading());
        grabSpecimenReady4.setZeroPowerAccelerationMultiplier(1.2);

        scoreSpecimen4 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 4)));
        scoreSpecimen4.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
        scoreSpecimen4.setZeroPowerAccelerationMultiplier(SLOW_ZERO_POWER_ACCEL);

        park = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 4), new Point(AutoConstants.SPECIMEN_PARK)));
        park.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_PARK.getHeading());
        park.setZeroPowerAccelerationMultiplier(4);
    }

    public void autonomousPathUpdate() {
        boolean robotInPos = MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getX(), follower.getPose().getX(), 1) &&
                MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getY(), follower.getPose().getY(), 1);

        switch (pathState) {
            case 0:
                currentPath = scorePreload;
                follower.followPath(currentPath);
                setActionState(0);
                setPathState(1);
                break;

            case 1:
                if(robotInPos || follower.getPose().getY() < 26) {
                    if(actionState == -1) {
                        if(outtake.getState().equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH)){
                            setActionState(1);
                        }
                        else if(ons){
                            currentPath = pushing;
                            follower.followPath(currentPath, true);
                            setActionState(13);
                            ons = false;
                        }
                        else {
                            currentPath = pushing0;
                            follower.followPath(currentPath, true);
                            setPathState(2);
                        }
                    }
                }
                break;

            case 2:
                if(robotInPos) {
                    currentPath = pushing1;
                    follower.followPath(currentPath, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(follower.getPose().getY() < 17) {
                    currentPath = pushing2;
                    follower.followPath(currentPath, true);
                    setPathState(4);
                }
                break;

            case 4:
                if(robotInPos) {
                    currentPath = pushing3;
                    follower.followPath(currentPath, true);
                    setPathState(5);
                }
                break;

            case 5:
                if(follower.getPose().getY() < 8) {
                    currentPath = pushing4;
                    follower.followPath(currentPath, true);
                    setPathState(6);
                }
                break;

            case 6:
                if(robotInPos) {
                    if(actionState == -1) {
                        currentPath = grabSpecimen1;
                        follower.followPath(currentPath, true);
                        setPathState(7);
                    }
                }
                break;

            case 7:
                if(robotInPos) {
                    if(actionState == -1 || pathTimer.getElapsedTimeSeconds() > 0.35) {
                        if(ons){
                            setActionState(0);
                            ons = false;
                            pathTimer.resetTimer();
                        }
                        else {
                            currentPath = scoreSpecimen1;
                            follower.followPath(currentPath, true);
                            setPathState(8);
                        }
                    }
                }
                break;

            case 8:
                if(robotInPos) {
                    if(actionState == -1) {
                        if (ons) {
                            setActionState(1);
                            ons = false;
                        }
                        else {
                            currentPath = grabSpecimenReady2;
                            follower.followPath(currentPath, true);
                            setActionState(13);
                            setPathState(9);
                        }
                    }
                }
                break;

            case 9:
                if(robotInPos) {
                    if(actionState == -1 || pathTimer.getElapsedTimeSeconds() > 0.35) {
                        if(ons){
                            setActionState(0);
                            ons = false;
                            pathTimer.resetTimer();
                        }
                        else {
                            currentPath = scoreSpecimen2;
                            follower.followPath(currentPath, true);
                            setPathState(10);
                        }
                    }
                }
                break;

            case 10:
                if(robotInPos) {
                    if(actionState == -1) {
                        if (ons) {
                            setActionState(1);
                            ons = false;
                        }
                        else {
                            currentPath = grabSpecimenReady3;
                            follower.followPath(currentPath, true);
                            setActionState(13);
                            setPathState(11);
                        }
                    }
                }
                break;

            case 11:
                if(robotInPos) {
                    if(actionState == -1 || pathTimer.getElapsedTimeSeconds() > 0.35) {
                        if(ons){
                            setActionState(0);
                            ons = false;
                            pathTimer.resetTimer();
                        }
                        else {
                            currentPath = scoreSpecimen3;
                            follower.followPath(currentPath, true);
                            setPathState(12);
                        }
                    }
                }
                break;

            case 12:
                if(robotInPos) {
                    if(actionState == -1) {
                        if (ons) {
                            setActionState(1);
                            ons = false;
                        }
                        else {
                            currentPath = grabSpecimenReady4;
                            follower.followPath(currentPath, true);
                            setActionState(13);
                            setPathState(13);
                        }
                    }
                }
                break;

            case 13:
                if(robotInPos) {
                    if(actionState == -1 || pathTimer.getElapsedTimeSeconds() > 0.35) {
                        if(ons){
                            setActionState(0);
                            ons = false;
                            pathTimer.resetTimer();
                        }
                        else {
                            currentPath = scoreSpecimen4;
                            follower.followPath(currentPath, true);
                            setPathState(14);
                        }
                    }
                }
                break;

            case 14:
                if(robotInPos) {
                    if(actionState == -1) {
                        if (ons) {
                            setActionState(1);
                            ons = false;
                        }
                        else {
                            currentPath = park;
                            follower.followPath(currentPath, true);
                            setActionState(2);
                            setPathState(-1);
                        }
                    }
                }
                break;
        }
    }
//TODO: Teleport to autoActionUpdate
    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                if(actionTimer.getElapsedTimeSeconds() > 0.15) {
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
                    setActionState(14);
                }
                break;

            case 1:
                if(actionTimer.getElapsedTimeSeconds() > 0.15) {
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
                    setActionState(14);
                }
                break;

            case 2:
                if(actionTimer.getElapsedTimeSeconds() > 0.5) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    setActionState(14);
                }
                break;

            case 13:
                if(actionTimer.getElapsedTimeSeconds() > 0) {
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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(AutoConstants.SPECIMEN_START);
        buildPaths();

        currentPath = scorePreload;

        intake.setState(IntakeConstants.START);
    }

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
            telemetry.addLine("initialized!");
            telemetry.update();
        }
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        intake.setState(IntakeConstants.START);
        setPathState(0);
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
