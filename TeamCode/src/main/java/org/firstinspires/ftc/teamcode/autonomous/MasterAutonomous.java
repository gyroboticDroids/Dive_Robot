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

@Autonomous(name = "Master Autonomous", group = "Autonomous", preselectTeleOp = "Master Tele-op")
public class MasterAutonomous extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private Intake intake;
    private Outtake outtake;

    //Config for auto
    boolean scoreSamples = false;
    boolean scoreSpecimens = false;
    boolean scoreBoth = false;

    private int startPos = 0;
    private boolean isPressed;
    private boolean onsFollowPath = false;

    private String lastOuttakeState;
    private String lastIntakeState;
    private int lastState;

    //Poses
    private Pose start;
    private final Pose specimenLeft = new Pose(36, 11.25, 0);
    private final Pose specimenRight = new Pose(114, 30, Math.toRadians(-180));
    private final Pose basketSample1 = new Pose(7, -50, Math.toRadians(25));
    private final Pose basketSample2 = new Pose(8, -51, Math.toRadians(13));
    private final Pose sample3 = new Pose(12, -45, Math.toRadians(-45));
    private final Pose basketSample3 = new Pose(7, -48, Math.toRadians(45));
    private final Pose touchBar = new Pose(72, -12, Math.toRadians(0));
    private final Pose observationZoneReady = new Pose(12, -72, Math.toRadians(0));
    private final Pose observationZone = new Pose(6, -72, Math.toRadians(0));

    //Paths
    private Path toLeftOfBar, toRightOfBar, toBasket1, toBasket2, toSample3, toBasket3, toBar, toObservationZoneReady, toObservationZone, toLeftBarFromObs;

    @Override
    public void init()
    {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        pathTimer = new Timer();
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        start = new Pose(138,30, Math.toRadians(-180));
        follower.setStartingPose(start);
        BuildPaths();
    }

    @Override
    public void init_loop()
    {
        telemetry.addLine("A B Y to change mode \nDpad left and right to change start pos");

        if(gamepad1.a)
        {
            scoreSamples = true;
            scoreSpecimens = false;
            scoreBoth = false;

            startPos = 0;
        }
        else if(gamepad1.b)
        {
            scoreSamples = false;
            scoreSpecimens = true;
            scoreBoth = false;

            startPos = 2;
        }
        else if(gamepad1.y)
        {
            scoreSamples = false;
            scoreSpecimens = false;
            scoreBoth = true;

            startPos = 1;
        }

        if(scoreSamples)
            telemetry.addLine("All Samples");
        if(scoreSpecimens)
            telemetry.addLine("All Specimens");
        if(scoreBoth)
            telemetry.addLine("1 Specimen and Samples");

        if(gamepad1.dpad_left && !isPressed)
        {
            startPos--;
            isPressed = true;
        }
        else if(gamepad1.dpad_right && !isPressed)
        {
            startPos++;
            isPressed = true;
        }
        else if(!gamepad1.dpad_left && !gamepad1.dpad_right)
        {
            isPressed = false;
        }

        startPos = (int) MathFunctions.clamp(startPos, 0, 3);

        telemetry.addData("Start Position Index", startPos);
        telemetry.update();
    }

//    @Override
//    public void start()
//    {
//        if(startPos == 0)
//        {
//            start = new Pose(7.125,-35.75,0);
//        }
//        else if(startPos == 1)
//        {
//            start = new Pose(7.125,-6.125,0);
//        }
//        else if(startPos == 2)
//        {
//
//        }
//        else if(startPos == 3)
//        {
//            start = new Pose(7.125,35.75,0);
//        }
//
//    }

    @Override
    public void loop()
    {
        autoPathUpdate();
        follower.update();
        intake.update();
        outtake.update();


        telemetry.addData("current state", pathState);
        telemetry.update();
    }

    private void BuildPaths()
    {
        toLeftOfBar = new Path(new BezierLine(new Point(start), new Point(specimenLeft)));
        toLeftOfBar.setLinearHeadingInterpolation(start.getHeading(), specimenLeft.getHeading());

        toRightOfBar = new Path(new BezierLine(new Point(start), new Point(specimenRight)));
        toRightOfBar.setLinearHeadingInterpolation(start.getHeading(), specimenRight.getHeading());

        toBasket1 = new Path(new BezierLine(new Point(specimenLeft), new Point(basketSample1)));
        toBasket1.setLinearHeadingInterpolation(specimenLeft.getHeading(), basketSample1.getHeading());

        toBasket2 = new Path(new BezierLine(new Point(basketSample1), new Point(basketSample2)));
        toBasket2.setLinearHeadingInterpolation(basketSample1.getHeading(), basketSample2.getHeading());

        toSample3 = new Path(new BezierLine(new Point(basketSample2), new Point(sample3)));
        toSample3.setLinearHeadingInterpolation(basketSample2.getHeading(), sample3.getHeading());

        toBasket3 = new Path(new BezierLine(new Point(sample3), new Point(basketSample3)));
        toBasket3.setLinearHeadingInterpolation(sample3.getHeading(), basketSample3.getHeading());

        toBar = new Path(new BezierCurve(new Point(basketSample3), new Point(new Pose(basketSample3.getX(), touchBar.getY())), new Point(touchBar)));
        toBar.setLinearHeadingInterpolation(basketSample3.getHeading(), touchBar.getHeading());

        toObservationZoneReady = new Path(new BezierLine(new Point(specimenRight), new Point(observationZoneReady)));
        toObservationZoneReady.setLinearHeadingInterpolation(specimenRight.getHeading(), observationZoneReady.getHeading());

        toObservationZone = new Path(new BezierLine(new Point(observationZoneReady), new Point(observationZone)));
        toObservationZone.setLinearHeadingInterpolation(observationZoneReady.getHeading(), observationZone.getHeading());

        toLeftBarFromObs = new Path(new BezierLine(new Point(observationZone), new Point(specimenLeft)));
        toLeftBarFromObs.setLinearHeadingInterpolation(observationZone.getHeading(), specimenLeft.getHeading());
    }

    private void autoPathUpdate()
    {
        switch (pathState)
        {
            case 0:
                if(scoreBoth)
                    setPathState(1);
                if(scoreSpecimens)
                    setPathState(30);
                if(scoreSamples)
                    setPathState(5);
                break;

            case 1:
                follower.followPath(toLeftOfBar, true);
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);

                if(!follower.isBusy() && !outtake.isBusy())
                {
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
                    setPathState(10);
                }
                break;

            case 5:
                follower.followPath(toBasket1, true);
                outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);

                if(!follower.isBusy() && !outtake.isBusy())
                {
                    outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                    setPathState(6);
                }
                break;

            case 10:
                if(!follower.isBusy() && !outtake.isBusy())
                {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);

                    if(lastState == 1) {
                        follower.followPath(toBasket1, true);
                    }
                    setPathState(11);
                }
                break;

            case 11:
                if(!follower.isBusy() && !outtake.isBusy())
                {
                    if(lastIntakeState.equals(IntakeConstants.INTAKE_SUB_READY)) {
                        intake.setState(IntakeConstants.INTAKE);
                        intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX);
                        setPathState(12);
                    }

                    intake.setState(IntakeConstants.INTAKE_SUB_READY);
                }
                break;

            case 12:
                if(!intake.isBusy())
                {
                    if(lastIntakeState.equals(IntakeConstants.TRANSFER)) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
                    setPathState(13);
                    }

                    intake.setState(IntakeConstants.TRANSFER);
                }
                break;

            case 13:
                if(!outtake.isBusy())
                {
                    if(lastOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
                        outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                        setPathState(14);
                    }

                    outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
                }
                break;

            case 14:
                if(!outtake.isBusy())
                {
                    follower.followPath(toBasket2);
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    intake.setState(IntakeConstants.INTAKE_SUB_READY);
                    setPathState(15);
                }
                break;

            case 15:
                if(!follower.isBusy() && !outtake.isBusy() && !intake.isBusy())
                {
                    intake.setState(IntakeConstants.INTAKE);
                    intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX);
                    setPathState(16);
                }
                break;

            case 16:
                if(!intake.isBusy())
                {
                    if (lastIntakeState.equals(IntakeConstants.TRANSFER)) {
                        outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
                        setPathState(17);
                    }

                    intake.setState(IntakeConstants.TRANSFER);
                }
                break;

            case 17:
                if(!outtake.isBusy())
                {
                    if(lastOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)) {
                        outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                        setPathState(18);
                    }

                    outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
                }
                break;

            case 18:
                if(!outtake.isBusy())
                {
                    follower.followPath(toSample3);
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    intake.setState(IntakeConstants.INTAKE_SUB_READY);
                    setPathState(19);
                }
                break;

            case 19:
                if(!intake.isBusy() && !follower.isBusy())
                {
                    intake.setState(IntakeConstants.INTAKE);
                    intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX);
                    setPathState(20);
                }
                break;

            case 20:
                if(!intake.isBusy())
                {
                    if(lastIntakeState.equals(IntakeConstants.TRANSFER)){
                        outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
                        follower.followPath(toBasket3);
                        setPathState(21);
                    }

                    intake.setState(IntakeConstants.TRANSFER);
                }
                break;

            case 21:
                if(!outtake.isBusy() && !follower.isBusy())
                {
                    if(lastOuttakeState.equals(OuttakeConstants.SCORE_SAMPLE_READY_HIGH)){
                        outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                        setPathState(22);
                    }

                    outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
                }
                break;

            case 22:
                if(!outtake.isBusy())
                {
                    follower.followPath(toBar);
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);

                    setPathState(100);
                }
                break;

            case 30:
                //if(pathTimer.getElapsedTimeSeconds() > 5)
                //{
                //    outtake.setState(OuttakeConstants.SCORE_SPECIMEN);

                //}

                //if(onsFollowPath){
                    follower.followPath(toRightOfBar, true);
                //    outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
                //}
                setPathState(100);
                break;

            case 31:
                if(!outtake.isBusy()) {
                    follower.followPath(toObservationZoneReady, true);
                    outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);

                    setPathState(32);
                }
                break;

            case 32:
                if(!follower.isBusy()) {
                    follower.followPath(toObservationZone, true);
                    setPathState(33);
                }
                break;

            case 33:
                if(!follower.isBusy()) {
                    if(!outtake.isBusy() && lastOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH))
                    {
                        follower.followPath(toLeftBarFromObs);
                        setPathState(34);
                    }

                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
                }
                break;

            case 34:
                if(!follower.isBusy()) {
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN);

                    setPathState(35);
                }
                break;

            case 35:
                if(!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    follower.followPath(toObservationZoneReady);
                    setPathState(100);
                }
                break;

            case 100:
                telemetry.addLine("Done!");
                break;
        }

        lastOuttakeState = outtake.getState();
        lastIntakeState = intake.getState();
        lastState = pathState;

        //outtake.update();
        //intake.update();

        onsFollowPath = false;
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        onsFollowPath = true;
    }
}
