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
    private int pathState;
    private int actionState;
    private Path currentPath;

    private Path scorePreload, intakeSpecimenLeft, outtakeSpecimenLeft, intakeSpecimenCenter, outtakeSpecimenCenter, intakeSpecimenRight, outtakeSpecimenRight;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_START), new Point(AutoConstants.SPECIMEN_SCORE)));
        scorePreload.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_START.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());

        intakeSpecimenLeft = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_SCORE), new Point(AutoConstants.SPECIMEN_INTAKE_LEFT.getX(), AutoConstants.SPECIMEN_SCORE.getY()), new Point(AutoConstants.SPECIMEN_INTAKE_LEFT)));
        intakeSpecimenLeft.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_INTAKE_LEFT.getHeading());

        outtakeSpecimenLeft = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_INTAKE_LEFT), new Point(AutoConstants.SPECIMEN_OUTTAKE_LEFT)));
        outtakeSpecimenLeft.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_INTAKE_LEFT.getHeading(), AutoConstants.SPECIMEN_OUTTAKE_LEFT.getHeading());

        intakeSpecimenCenter = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_OUTTAKE_RIGHT), new Point(AutoConstants.SPECIMEN_INTAKE_CENTER)));
        intakeSpecimenCenter.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_OUTTAKE_RIGHT.getHeading(), AutoConstants.SPECIMEN_INTAKE_CENTER.getHeading());

        outtakeSpecimenCenter = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_INTAKE_CENTER), new Point(AutoConstants.SPECIMEN_OUTTAKE_CENTER)));
        outtakeSpecimenCenter.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_INTAKE_CENTER.getHeading(), AutoConstants.SPECIMEN_OUTTAKE_CENTER.getHeading());

        intakeSpecimenRight = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_OUTTAKE_CENTER), new Point(AutoConstants.SPECIMEN_INTAKE_RIGHT)));
        intakeSpecimenRight.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_OUTTAKE_CENTER.getHeading(), AutoConstants.SPECIMEN_INTAKE_RIGHT.getHeading());

        outtakeSpecimenRight = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_INTAKE_RIGHT), new Point(AutoConstants.SPECIMEN_OUTTAKE_RIGHT)));
        outtakeSpecimenRight.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_INTAKE_RIGHT.getHeading(), AutoConstants.SPECIMEN_OUTTAKE_RIGHT.getHeading());
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

                if(robotInPos){
                    /* Score Preload */

                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                        currentPath = intakeSpecimenLeft;
                        follower.followPath(currentPath, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(robotInPos) {
                    /* Grab Sample */

                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                        currentPath = outtakeSpecimenLeft;
                        follower.followPath(currentPath, true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(robotInPos) {
                    /* Score Sample */

                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                        currentPath = intakeSpecimenCenter;
                        follower.followPath(currentPath, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(robotInPos) {
                    /* Grab Sample */

                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                       currentPath = outtakeSpecimenCenter;
                        follower.followPath(currentPath, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(robotInPos) {
                    /* Score Sample */

                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                        currentPath = intakeSpecimenRight;
                        follower.followPath(currentPath, true);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if(robotInPos) {
                    /* Grab Sample */

                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                        currentPath = outtakeSpecimenRight;
                        follower.followPath(currentPath, true);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if(robotInPos) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                      //  currentPath = touchBar;
                        follower.followPath(currentPath,true);
                        setPathState(8);
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
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
                setActionState(-1);
                break;

            case 1:
                if(!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
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
        //follower.setStartingPose(AutoConstants.START_POSE);
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
