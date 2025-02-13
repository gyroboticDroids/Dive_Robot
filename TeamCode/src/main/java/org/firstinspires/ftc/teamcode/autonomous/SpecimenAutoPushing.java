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

@Autonomous(name = "4 specimen auto pushing", group = "autonomous", preselectTeleOp = "Master Tele-op")
public class SpecimenAutoPushing extends OpMode {
    private int slideRangeSubtract = 200;

    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;
    private Intake intake;
    private Outtake outtake;
    private int pathState = -1;
    private int actionState = -1;
    private boolean onsIntakeOut = false;
    private boolean onsGrabSample = false;
    private Path currentPath;

    private Path scorePreload, grabSpecimen, scoreSpecimen1, grabSpecimen1,
            grabSpecimenReady2, scoreSpecimen2, grabSpecimenReady3, scoreSpecimen3, grabSpecimenReady4;

    private PathChain pushing;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_START), new Point(AutoConstants.SPECIMEN_SCORE)));
        scorePreload.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_START.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
        scorePreload.setZeroPowerAccelerationMultiplier(2);

        pushing = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(AutoConstants.SPECIMEN_SCORE), AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT1 ,new Point(AutoConstants.SPECIMEN_PUSHING1)))
                .setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_PUSHING1.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .addPath(new BezierCurve(new Point(AutoConstants.SPECIMEN_PUSHING1), AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT2 ,new Point(AutoConstants.SPECIMEN_PUSHING2)))
                .setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING1.getHeading(), AutoConstants.SPECIMEN_PUSHING2.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .addPath(new BezierLine(new Point(AutoConstants.SPECIMEN_PUSHING2), new Point(AutoConstants.SPECIMEN_PUSHING3)))
                .setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING2.getHeading(), AutoConstants.SPECIMEN_PUSHING3.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .addPath(new BezierCurve(new Point(AutoConstants.SPECIMEN_PUSHING3), AutoConstants.SPECIMEN_PUSHING_CONTROL_POINT4 ,new Point(AutoConstants.SPECIMEN_PUSHING4)))
                .setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING3.getHeading(), AutoConstants.SPECIMEN_PUSHING4.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .addPath(new BezierLine(new Point(AutoConstants.SPECIMEN_PUSHING4), new Point(AutoConstants.SPECIMEN_PUSHING5)))
                .setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING4.getHeading(), AutoConstants.SPECIMEN_PUSHING5.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        grabSpecimen1 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_PUSHING5), new Point(AutoConstants.SPECIMEN_GRAB_2)));
        grabSpecimen1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_PUSHING5.getHeading(), AutoConstants.SPECIMEN_GRAB_2.getHeading());
        grabSpecimen1.setZeroPowerAccelerationMultiplier(1.5);

        grabSpecimen = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_GRAB_READY), new Point(AutoConstants.SPECIMEN_GRAB)));
        grabSpecimen.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB_READY.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading());

        scoreSpecimen1 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB_2), AutoConstants.SPECIMEN_CONTROL_POINT, new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 1, AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 1)));
        scoreSpecimen1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB_2.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
        scoreSpecimen1.setZeroPowerAccelerationMultiplier(1.2);

        grabSpecimenReady2 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 1, AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 1), new Point(AutoConstants.SPECIMEN_GRAB_READY)));
        grabSpecimenReady2.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB_READY.getHeading());
        grabSpecimenReady2.setZeroPowerAccelerationMultiplier(1.5);

        scoreSpecimen2 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 2, AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 2)));
        scoreSpecimen2.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
        scoreSpecimen2.setZeroPowerAccelerationMultiplier(1.5);

        grabSpecimenReady3 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 3, AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 2), new Point(AutoConstants.SPECIMEN_GRAB_READY)));
        grabSpecimenReady3.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB_READY.getHeading());
        grabSpecimenReady3.setZeroPowerAccelerationMultiplier(1.5);

        scoreSpecimen3 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 3, AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 3)));
        scoreSpecimen3.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
        scoreSpecimen3.setZeroPowerAccelerationMultiplier(1.5);

        grabSpecimenReady4 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 3, AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 3), new Point(AutoConstants.SPECIMEN_GRAB_READY.getX() - 3, AutoConstants.SPECIMEN_GRAB_READY.getY() - 3)));
        grabSpecimenReady4.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB_READY.getHeading());
        grabSpecimenReady4.setZeroPowerAccelerationMultiplier(1.5);
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
                if(robotInPos) {
                    if(actionState == -1) {
                        if(outtake.getState().equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH)){
                            setActionState(1);
                        }
                        else {
                            currentPath = pushing.getPath(4);
                            follower.followPath(pushing, true);
                            setActionState(13);
                            setPathState(7);
                        }
                    }
                }
                break;

            case 7:
                if(robotInPos) {
                    if(actionState == -1) {
                        currentPath = grabSpecimen1;
                        follower.followPath(currentPath, true);
                        setPathState(9);
                    }
                }
                break;

            case 9:
                if(robotInPos) {
                    if(actionState == -1 || actionTimer.getElapsedTimeSeconds() > 0.5) {
                        if(onsIntakeOut){
                            setActionState(0);
                            onsIntakeOut = false;
                            actionTimer.resetTimer();
                        }
                        else {
                            currentPath = scoreSpecimen1;
                            follower.followPath(currentPath, true);
                            setPathState(10);
                        }
                    }
                }
                break;

            case 10:
                if(robotInPos) {
                    if(actionState == -1) {
                        if (onsIntakeOut) {
                            setActionState(1);
                            onsIntakeOut = false;
                        }
                        else {
                            currentPath = grabSpecimenReady2;
                            follower.followPath(currentPath, true);
                            setActionState(13);
                            setPathState(11);
                        }
                    }
                }
                break;

            case 11:
                if(robotInPos) {
                    if(onsIntakeOut){
                        pathTimer.resetTimer();
                        onsIntakeOut = false;
                    }

                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        currentPath = grabSpecimen;
                        follower.followPath(currentPath, true);
                        setPathState(12);
                    }
                }
                break;

            case 12:
                if(robotInPos) {
                    if(actionState == -1 || actionTimer.getElapsedTimeSeconds() > 0.5) {
                        if(onsIntakeOut){
                            setActionState(0);
                            onsIntakeOut = false;
                            actionTimer.resetTimer();
                        }
                        else {
                            currentPath = scoreSpecimen2;
                            follower.followPath(currentPath, true);
                            setPathState(13);
                        }
                    }
                }
                break;

            case 13:
                if(robotInPos) {
                    if(actionState == -1) {
                        if (onsIntakeOut) {
                            setActionState(1);
                            onsIntakeOut = false;
                        }
                        else {
                            currentPath = grabSpecimenReady3;
                            follower.followPath(currentPath, true);
                            setActionState(13);
                            setPathState(14);
                        }
                    }
                }
                break;

            case 14:
                if(robotInPos) {
                    if(onsIntakeOut){
                        pathTimer.resetTimer();
                        onsIntakeOut = false;
                    }

                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        currentPath = grabSpecimen;
                        follower.followPath(currentPath, true);
                        setPathState(15);
                    }
                }
                break;

            case 15:
                if(robotInPos) {
                    if(actionState == -1 || actionTimer.getElapsedTimeSeconds() > 0.5) {
                        if(onsIntakeOut){
                            setActionState(0);
                            onsIntakeOut = false;
                            actionTimer.resetTimer();
                        }
                        else {
                            currentPath = scoreSpecimen3;
                            follower.followPath(currentPath, true);
                            setPathState(16);
                        }
                    }
                }
                break;

            case 16:
                if(robotInPos) {
                    if(actionState == -1) {
                        if (onsIntakeOut) {
                            setActionState(1);
                            onsIntakeOut = false;
                        }
                        else {
                            currentPath = grabSpecimenReady4;
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
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
                setActionState(14);
                break;

            case 1:
                if(actionTimer.getElapsedTimeSeconds() > 0.35) {
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
                    setActionState(14);
                }
                break;

            case 2:
                if(!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    setActionState(14);
                }
                break;

            case 10:
                intake.setState(IntakeConstants.INTAKE_SUB_READY);
                setActionState(11);
                break;

            case 11:
                if(!intake.isBusy()) {
                    intake.setState(IntakeConstants.INTAKE);
                    intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX - slideRangeSubtract);
                    setActionState(12);
                }
            break;

            case 12:
                if(intake.getHorizontalSlidePos() > IntakeConstants.SLIDES_MAX - slideRangeSubtract - 50) {
                    if(onsGrabSample){
                        actionTimer.resetTimer();
                        onsGrabSample = false;
                    }

                    if(actionTimer.getElapsedTimeSeconds() > 0.25) {
                        intake.setState(IntakeConstants.TRANSFER);
                        setActionState(13);
                    }
                }
                break;

            case 13:
                if(actionTimer.getElapsedTimeSeconds() > 0.25) {
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
        onsIntakeOut = true;
        pathTimer.resetTimer();
    }

    public void setActionState(int aState) {
        actionState = aState;
        onsGrabSample = true;
        actionTimer.resetTimer();
    }

    @Override
    public void init()
    {
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
        telemetry.addData("hori slide pos", intake.getHorizontalSlidePos());
        telemetry.addData("hori slide setpoint", intake.getHorizontalPosition());
        telemetry.addData("intake is busy", intake.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
