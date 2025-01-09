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

@Autonomous(name = "5 specimen auto", group = "autonomous", preselectTeleOp = "Master Tele-op")
public class SpecimenAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;
    private Intake intake;
    private Outtake outtake;
    private int pathState = -1;
    private int actionState = -1;
    private Path currentPath;

    private Path scorePreload, grabSpecimen, scoreSpecimen1, transferSpecimenLeft, transferSpecimenCenter, intakeSpecimenRight, transferSpecimenRight, grabSpecimenReady1,
            grabSpecimenReady2, scoreSpecimen2, grabSpecimenReady3, scoreSpecimen3, grabSpecimenReady4, scoreSpecimen4, grabSpecimenReady5;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_START), new Point(AutoConstants.SPECIMEN_SCORE)));
        scorePreload.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_START.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());

        transferSpecimenLeft = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE), new Point(AutoConstants.SPECIMEN_TRANSFER_LEFT)));
        transferSpecimenLeft.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_TRANSFER_LEFT.getHeading());

        transferSpecimenCenter = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_TRANSFER_LEFT), new Point(AutoConstants.SPECIMEN_TRANSFER_CENTER)));
        transferSpecimenCenter.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_TRANSFER_LEFT.getHeading(), AutoConstants.SPECIMEN_TRANSFER_CENTER.getHeading());

        intakeSpecimenRight = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_TRANSFER_CENTER), new Point(AutoConstants.SPECIMEN_INTAKE_RIGHT)));
        intakeSpecimenRight.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_TRANSFER_CENTER.getHeading(), AutoConstants.SPECIMEN_INTAKE_RIGHT.getHeading());

        transferSpecimenRight = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_INTAKE_RIGHT), new Point(AutoConstants.SPECIMEN_TRANSFER_CENTER)));
        transferSpecimenRight.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_INTAKE_RIGHT.getHeading(), AutoConstants.SPECIMEN_TRANSFER_CENTER.getHeading());

        grabSpecimenReady1 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_TRANSFER_CENTER), new Point(AutoConstants.SPECIMEN_GRAB_READY)));
        grabSpecimenReady1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_TRANSFER_CENTER.getHeading(), AutoConstants.SPECIMEN_GRAB_READY.getHeading());

        grabSpecimen = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_GRAB_READY), new Point(AutoConstants.SPECIMEN_GRAB)));
        grabSpecimen.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB_READY.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading());

        scoreSpecimen1 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_GRAB.getY()),
                new Point(AutoConstants.SPECIMEN_GRAB.getX(), AutoConstants.SPECIMEN_SCORE.getY()), new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY())));
        scoreSpecimen1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());

        grabSpecimenReady2 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY()), new Point(AutoConstants.SPECIMEN_GRAB_READY)));
        grabSpecimenReady2.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB_READY.getHeading());

        scoreSpecimen2 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_GRAB.getY() + AutoConstants.Y_INCREMENT * 1),
                new Point(AutoConstants.SPECIMEN_GRAB.getX(), AutoConstants.SPECIMEN_SCORE.getY()), new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 1)));
        scoreSpecimen2.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());

        grabSpecimenReady3 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 1), new Point(AutoConstants.SPECIMEN_GRAB_READY)));
        grabSpecimenReady3.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB_READY.getHeading());

        scoreSpecimen3 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_GRAB.getY() + AutoConstants.Y_INCREMENT * 2),
                new Point(AutoConstants.SPECIMEN_GRAB.getX(), AutoConstants.SPECIMEN_SCORE.getY()), new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 2)));
        scoreSpecimen3.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());

        grabSpecimenReady4 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 2), new Point(AutoConstants.SPECIMEN_GRAB_READY)));
        grabSpecimenReady4.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB_READY.getHeading());

        scoreSpecimen4 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB), new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_GRAB.getY() + AutoConstants.Y_INCREMENT * 3),
                new Point(AutoConstants.SPECIMEN_GRAB.getX(), AutoConstants.SPECIMEN_SCORE.getY()), new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 3)));
        scoreSpecimen4.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());

        grabSpecimenReady5 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + AutoConstants.Y_INCREMENT * 3), new Point(AutoConstants.SPECIMEN_GRAB_READY)));
        grabSpecimenReady5.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB_READY.getHeading());
    }

    public void autonomousPathUpdate() {
        boolean robotInPos = MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getX(), follower.getPose().getX(), 1) &&
                MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getY(), follower.getPose().getY(), 1);

        switch (pathState) {
            case 0:
                currentPath = scorePreload;
                follower.followPath(currentPath, true);
                //setActionState(0);
                setPathState(1);
                break;

            case 1:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = transferSpecimenLeft;
                        follower.followPath(currentPath, true);
                        setPathState(2);
                    }
                }

//                if (robotInPos) {
//                    setActionState(1);
//                }
                break;

            case 2:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = transferSpecimenCenter;
                        follower.followPath(currentPath, true);
                        setPathState(3);
                    }
                }
                break;

            case 3:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = intakeSpecimenRight;
                        follower.followPath(currentPath, true);
                        setPathState(4);
                    }
                }
                break;

            case 4:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = transferSpecimenRight;
                        follower.followPath(currentPath, true);
                        setPathState(7);
                    }
                }
                break;

//            case 5:
//                if(robotInPos) {
//                    if(actionTimer.getElapsedTimeSeconds() > 2) {
//                        currentPath = grabSpecimenReady1;
//                        follower.followPath(currentPath, true);
//                        setPathState(6);
//                    }
//                }
//                break;
//
//            case 6:
//                if(robotInPos) {
//                    if(actionTimer.getElapsedTimeSeconds() > 2) {
//                        currentPath = outtakeSpecimenRight;
//                        follower.followPath(currentPath, true);
//                        setPathState(7);
//                    }
//                }
//                break;

            case 7:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = grabSpecimenReady1;
                        follower.followPath(currentPath, true);
                        setPathState(8);
                    }
                }
                break;

            case 8:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = grabSpecimen;
                        follower.followPath(currentPath, true);
                        setPathState(9);
                    }
                }
                break;

            case 9:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = scoreSpecimen1;
                        follower.followPath(currentPath, true);
                        setPathState(10);
                    }
                }
                break;

            case 10:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = grabSpecimenReady2;
                        follower.followPath(currentPath, true);
                        setPathState(11);
                    }
                }
                break;

            case 11:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = grabSpecimen;
                        follower.followPath(currentPath, true);
                        setPathState(12);
                    }
                }
                break;

            case 12:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = scoreSpecimen2;
                        follower.followPath(currentPath, true);
                        setPathState(13);
                    }
                }
                break;

            case 13:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = grabSpecimenReady3;
                        follower.followPath(currentPath, true);
                        setPathState(14);
                    }
                }
                break;

            case 14:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = grabSpecimen;
                        follower.followPath(currentPath, true);
                        setPathState(15);
                    }
                }
                break;

            case 15:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = scoreSpecimen3;
                        follower.followPath(currentPath, true);
                        setPathState(16);
                    }
                }
                break;

            case 16:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = grabSpecimenReady4;
                        follower.followPath(currentPath, true);
                        setPathState(17);
                    }
                }
                break;

            case 17:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = grabSpecimen;
                        follower.followPath(currentPath, true);
                        setPathState(18);
                    }
                }
                break;

            case 18:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = scoreSpecimen4;
                        follower.followPath(currentPath, true);
                        setPathState(19);
                    }
                }
                break;

            case 19:
                if(robotInPos) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        currentPath = grabSpecimenReady5;
                        follower.followPath(currentPath, true);
                        setPathState(20);
                    }
                }
                break;
        }
    }
//TODO:
    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
                setActionState(-1);
                break;

            case 1:
                if(!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
                    setActionState(10);
                }
                break;

            case 10:
                if(!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
                    setActionState(-1);
                }

                break;

            case 11:
                intake.setState(IntakeConstants.INTAKE_SUB_READY);
                setActionState(12);
            break;

            case 12:
                intake.setState(IntakeConstants.REJECT);
                setActionState(13);
                break;

            case 13:
                if(!intake.isBusy()) {
                    intake.setState(IntakeConstants.INTAKE);
                    intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX);
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
        actionTimer.resetTimer();
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
        follower.setStartingPose(AutoConstants.SPECIMEN_START);
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
