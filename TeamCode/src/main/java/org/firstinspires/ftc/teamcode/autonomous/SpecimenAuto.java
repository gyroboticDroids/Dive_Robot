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

@Autonomous(name = "5 specimen auto intake and outtake", group = "autonomous", preselectTeleOp = "Master Tele-op")
public class SpecimenAuto extends OpMode {
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
    private boolean robotInPos;
    private Path currentPath;
    private double currentHeading;

    private Path scorePreload, grabSpecimen, scoreSpecimen1, intakeSpecimenLeft, intakeSpecimenCenter, intakeSpecimenRight, grabSpecimenReady1,
            grabSpecimenReady2, scoreSpecimen2, grabSpecimenReady3, scoreSpecimen3, grabSpecimenReady4, scoreSpecimen4, grabSpecimenReady5;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_START), new Point(AutoConstants.SPECIMEN_SCORE)));
        scorePreload.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_START.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());

        intakeSpecimenRight = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_SCORE), new Point(AutoConstants.SPECIMEN_INTAKE_RIGHT.getX(), AutoConstants.SPECIMEN_SCORE.getY()),
                new Point(AutoConstants.SPECIMEN_INTAKE_RIGHT)));
        intakeSpecimenRight.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_INTAKE_RIGHT.getHeading());

        intakeSpecimenCenter = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_OUTTAKE_RIGHT), new Point(AutoConstants.SPECIMEN_INTAKE_CENTER)));
        intakeSpecimenCenter.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_OUTTAKE_RIGHT.getHeading(), AutoConstants.SPECIMEN_INTAKE_CENTER.getHeading());

        intakeSpecimenLeft = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_OUTTAKE_CENTER), AutoConstants.SPECIMEN_CONTROL_POINT1 ,new Point(AutoConstants.SPECIMEN_INTAKE_LEFT)));
        intakeSpecimenLeft.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_OUTTAKE_CENTER.getHeading(), AutoConstants.SPECIMEN_INTAKE_LEFT.getHeading());

        grabSpecimenReady1 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_OUTTAKE_LEFT), new Point(AutoConstants.SPECIMEN_GRAB_READY)));
        grabSpecimenReady1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_OUTTAKE_LEFT.getHeading(), AutoConstants.SPECIMEN_GRAB_READY.getHeading());

        grabSpecimen = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_GRAB_READY), new Point(AutoConstants.SPECIMEN_GRAB)));
        grabSpecimen.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB_READY.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading());

        scoreSpecimen1 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 1, AutoConstants.SPECIMEN_GRAB.getY() + AutoConstants.Y_INCREMENT * 1),
                new Point(AutoConstants.SPECIMEN_GRAB.getX(), AutoConstants.SPECIMEN_SCORE.getY()), new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 1, AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 1)));
        scoreSpecimen1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
        scoreSpecimen1.setZeroPowerAccelerationMultiplier(1.25);

        grabSpecimenReady2 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 1, AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 1), new Point(AutoConstants.SPECIMEN_GRAB_READY)));
        grabSpecimenReady2.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB_READY.getHeading());

        scoreSpecimen2 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 2, AutoConstants.SPECIMEN_GRAB.getY() + AutoConstants.Y_INCREMENT * 2),
                new Point(AutoConstants.SPECIMEN_GRAB.getX(), AutoConstants.SPECIMEN_SCORE.getY()), new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 2, AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 2)));
        scoreSpecimen2.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
        scoreSpecimen2.setZeroPowerAccelerationMultiplier(1.25);

        grabSpecimenReady3 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 3, AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 2), new Point(AutoConstants.SPECIMEN_GRAB_READY)));
        grabSpecimenReady3.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB_READY.getHeading());

        scoreSpecimen3 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 3, AutoConstants.SPECIMEN_GRAB.getY() + AutoConstants.Y_INCREMENT * 3),
                new Point(AutoConstants.SPECIMEN_GRAB.getX(), AutoConstants.SPECIMEN_SCORE.getY()), new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 3, AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 3)));
        scoreSpecimen3.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());

        grabSpecimenReady4 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 3, AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 3), new Point(AutoConstants.SPECIMEN_GRAB_READY)));
        grabSpecimenReady4.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB_READY.getHeading());

        scoreSpecimen4 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 4, AutoConstants.SPECIMEN_GRAB.getY() + AutoConstants.Y_INCREMENT * 4),
                new Point(AutoConstants.SPECIMEN_GRAB.getX(), AutoConstants.SPECIMEN_SCORE.getY()), new Point(AutoConstants.SPECIMEN_SCORE.getX() + AutoConstants.X_INCREMENT * 4, AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 4)));
        scoreSpecimen4.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());

        grabSpecimenReady5 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 4), new Point(AutoConstants.SPECIMEN_GRAB_READY)));
        grabSpecimenReady5.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB_READY.getHeading());
    }

    public void autonomousPathUpdate() {
        robotInPos = MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getX(), follower.getPose().getX(), 1) &&
                MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getY(), follower.getPose().getY(), 1) &&
                MathFunctions.roughlyEquals(currentHeading, follower.getPose().getHeading(), Math.toRadians(5));

        switch (pathState) {
            case 0:
                currentPath = scorePreload;
                currentHeading = currentPath.getHeadingGoal(1);
                follower.followPath(currentPath, true);
                setActionState(0);
                setPathState(1);
                break;

            case 1:
                if(robotInPos) {
                    if(actionState == -1) {
                        if (onsIntakeOut) {
                            setActionState(1);
                            onsIntakeOut = false;
                        } else {
                            currentPath = intakeSpecimenRight;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            setActionState(5);
                            setPathState(2);
                        }
                    }
                }
                break;

            case 2:
                if(robotInPos) {
                    if(actionState == -1 || actionState == 13) {
                        if (onsIntakeOut) {
                            setActionState(10);
                            onsIntakeOut = false;
                        } else {
                            currentHeading = AutoConstants.SPECIMEN_OUTTAKE_RIGHT.getHeading();
                            follower.holdPoint(AutoConstants.SPECIMEN_OUTTAKE_RIGHT);
                            setPathState(3);
                        }
                    }
                }
                break;

            case 3:
                if(robotInPos) {
                    if(actionState == -1) {
                        if (onsIntakeOut) {
                            setActionState(15);
                            onsIntakeOut = false;
                        } else {
                            follower.setMaxPower(0.5);
                            currentPath = intakeSpecimenCenter;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            setPathState(4);
                        }
                    }
                }
                break;

            case 4:
                if(robotInPos) {
                    if(actionState == -1 || actionState == 13) {
                        if (onsIntakeOut) {
                            setActionState(10);
                            onsIntakeOut = false;
                        } else {
                            follower.setMaxPower(1);
                            currentHeading = AutoConstants.SPECIMEN_OUTTAKE_CENTER.getHeading();
                            follower.holdPoint(AutoConstants.SPECIMEN_OUTTAKE_CENTER);
                            setPathState(5);
                        }
                    }
                }
                break;

            case 5:
                if(robotInPos) {
                    if(actionState == -1) {
                        if (onsIntakeOut) {
                            setActionState(15);
                            onsIntakeOut = false;
                        } else {
                            follower.setMaxPower(0.5);
                            currentPath = intakeSpecimenLeft;
                            currentHeading = currentPath.getHeadingGoal(1);
                            follower.followPath(currentPath, true);
                            setPathState(6);
                        }
                    }
                }
                break;

            case 6:
                if(robotInPos) {
                    if(actionState == -1 || actionState == 13) {
                        if (onsIntakeOut) {
                            setActionState(10);
                            onsIntakeOut = false;
                        } else {
                            follower.setMaxPower(1);
                            currentHeading = AutoConstants.SPECIMEN_OUTTAKE_LEFT.getHeading();
                            follower.holdPoint(AutoConstants.SPECIMEN_OUTTAKE_LEFT);
                            setPathState(-1);
                        }
                    }
                }
                break;

            case 7:
                if(robotInPos && actionTimer.getElapsedTimeSeconds() > 3) {
                    if(actionState == -1) {
                        if (onsIntakeOut) {
                            setActionState(10);
                            onsIntakeOut = false;
                        } else {
                            currentPath = grabSpecimenReady1;
                            follower.followPath(currentPath, true);
                            setPathState(8);
                        }
                    }
                }
                break;

            case 8:
                if(robotInPos) {
                    if(onsIntakeOut){
                        pathTimer.resetTimer();
                        onsIntakeOut = false;
                    }

                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        currentPath = grabSpecimen;
                        follower.followPath(currentPath, true);
                        setPathState(9);
                    }
                }
                break;

            case 9:
                if(robotInPos) {
                    if(actionState == -1) {
                        if(onsIntakeOut){
                            setActionState(0);
                            onsIntakeOut = false;
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
                    if(actionState == -1) {
                        if(onsIntakeOut){
                            setActionState(0);
                            onsIntakeOut = false;
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
                setActionState(20);
                break;

            case 1:
                if(actionTimer.getElapsedTimeSeconds() > 0.5) {
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
                    setActionState(20);
                }
                break;

            case 5:
                if(!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    setActionState(20);
                }
                break;

            case 10:
                intake.setState(IntakeConstants.INTAKE_SUB_READY);
                setActionState(11);
                break;

            case 11:
                if(!intake.isBusy()) {
                    intake.setState(IntakeConstants.INTAKE);
                    intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX - 100);
                    setActionState(12);
                }
            break;

            case 12:
                if(MathFunctions.roughlyEquals(intake.getHorizontalSlidePos(), intake.getHorizontalPosition(), IntakeConstants.SLIDES_ACCURACY)) {
                    if(onsGrabSample) {
                        actionTimer.resetTimer();
                        onsGrabSample = false;
                    }

                    if(actionTimer.getElapsedTimeSeconds() > 1) {
                        intake.setHorizontalPosition(IntakeConstants.SLIDES_OUT);
                        setActionState(13);
                    }
                }
                break;

            case 13:
                if(!intake.isBusy() && MathFunctions.roughlyEquals(intake.getHorizontalSlidePos(), intake.getHorizontalPosition(), IntakeConstants.SLIDES_ACCURACY)) {
                    setActionState(-1);
                }
                break;

            case 20:
                if(!outtake.isBusy()) {
                    setActionState(-1);
                }
                break;

            case 15:
                if(onsGrabSample) {
                    intake.setState(IntakeConstants.CLEAR_SUB);
                    onsGrabSample = false;
                }

                if(!intake.isBusy() && actionTimer.getElapsedTimeSeconds() > 2) {
                    setActionState(13);
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
            telemetry.update();
        }
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
