package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Outtake;

@Autonomous(name = "Master Autonomous", group = "Autonomous", preselectTeleOp = "Master Tele-op")
public class MasterAutonomousOliver extends OpMode {
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

    private String lastOuttakeState;
    private String lastIntakeState;

    //Poses
    private Pose start;
    private final Pose specimenLeft = new Pose(36, 11.25, 0);
    private final Pose basketSample1 = new Pose(7, -50, Math.toRadians(25));
    private final Pose basketSample2 = new Pose(8, -51, Math.toRadians(13));
    private final Pose sample3 = new Pose(12, -45, Math.toRadians(-45));
    private final Pose basketSample3 = new Pose(7, -48, Math.toRadians(45));
    private final Pose touchBar = new Pose(72, -12, Math.toRadians(0));
    private final Pose specimenRight = new Pose(36, -11.25, 0);
    private final Pose pushingReady1 = new Pose(24, -24, 0);
    private final Pose pushingReady2 = new Pose(48, -36, 0);
    private final Pose pushingReady3 = new Pose(60, -48, 0);
    private final Pose pushing1 = new Pose(12, -48, 0);
    private final Pose pushing2 = new Pose(36, -36, 0);
    private final Pose pushing3 = new Pose(36, -36, 0);
    private final Pose grabSpecimen = new Pose(0, -36, 0);

    //Paths
    private Path toLeftOfBar, toBasket1, toBasket2, toSample3, toBasket3, toBar, toRightOfBar, toRedSampleIntake1,
            toRedSampleIntake2, toRedSampleOuttake3, toRedSampleOuttake1, toRedSampleOuttake2, toRedSampleIntake3, toGrabSpecimen;

    @Override
    public void init()
    {
        follower = new Follower(hardwareMap);
        pathTimer = new Timer();
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
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

    @Override
    public void start()
    {
        if(startPos == 0)
        {
            start = new Pose(0,-35.75,0);
        }
        else if(startPos == 1)
        {
            start = new Pose(0,-11.25,0);
        }
        else if(startPos == 2)
        {
            start = new Pose(0,11.25,0);
        }
        else if(startPos == 3)
        {
            start = new Pose(0,35.75,0);
        }

        follower.setStartingPose(start);

        BuildPaths();
    }

    @Override
    public void loop()
    {
        follower.update();
        intake.Update();
        outtake.Update();
        AutoPathUpdate();

        telemetry.addData("current state", pathState);
        telemetry.update();
    }

    private void BuildPaths()
    {
        toLeftOfBar = new Path(new BezierLine(new Point(start), new Point(specimenLeft)));
        toLeftOfBar.setLinearHeadingInterpolation(start.getHeading(), specimenLeft.getHeading());

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

        toRightOfBar = new Path(new BezierLine(new Point(start), new Point(specimenRight)));
        toRightOfBar.setLinearHeadingInterpolation(start.getHeading(), specimenRight.getHeading());

        toRedSampleIntake1 = new Path(new BezierLine(new Point(specimenRight), new Point(pushingReady1)));
        toRedSampleIntake1.setLinearHeadingInterpolation(specimenRight.getHeading(), pushingReady1.getHeading());

        toRedSampleIntake2 = new Path(new BezierLine(new Point(pushing1), new Point(pushingReady2)));
        toRedSampleIntake2.setLinearHeadingInterpolation(pushing1.getHeading(), pushingReady2.getHeading());

        toRedSampleIntake3 = new Path(new BezierLine(new Point(pushing2), new Point(pushingReady3)));
        toRedSampleIntake3.setLinearHeadingInterpolation(pushing2.getHeading(), pushingReady3.getHeading());

        toRedSampleOuttake1 = new Path(new BezierLine(new Point(pushingReady1), new Point(pushing1)));
        toRedSampleOuttake1.setLinearHeadingInterpolation(pushingReady1.getHeading(), pushing1.getHeading());

        toRedSampleOuttake2 = new Path(new BezierLine(new Point(pushingReady2), new Point(pushing2)));
        toRedSampleOuttake2.setLinearHeadingInterpolation(pushingReady2.getHeading(), pushing2.getHeading());

        toRedSampleOuttake3 = new Path(new BezierLine(new Point(pushingReady3), new Point(pushing3)));
        toRedSampleOuttake3.setLinearHeadingInterpolation(pushingReady3.getHeading(), pushing3.getHeading());

        toGrabSpecimen = new Path(new BezierLine(new Point(pushing3), new Point(grabSpecimen)));
        toGrabSpecimen.setLinearHeadingInterpolation(pushing3.getHeading(), grabSpecimen.getHeading());
    }

    private void AutoPathUpdate()
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

                if(!follower.isBusy() && !outtake.IsBusy())
                {
                    if(lastOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH) && !outtake.IsBusy())
                    {
                        outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
                    }
                }
                setPathState(2);
                break;

            case 30:
                follower.followPath(toRightOfBar, true);
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);

                if(!follower.isBusy() && !outtake.IsBusy())
                {
                    if(lastOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH) && !outtake.IsBusy())
                    {
                        outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
                    }
                }
                setPathState(31);
                break;

            case 31:
                if(lastOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN) && !outtake.IsBusy())
                {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    follower.followPath(toRedSampleIntake1, true);
                }
                setPathState(32);
                break;

            case 32:
                if(lastOuttakeState.equals(OuttakeConstants.TRANSFER_INTAKE_READY) && !outtake.IsBusy())
                {
                    intake.setState(IntakeConstants.INTAKE_SUB_READY);

                    if(lastIntakeState.equals(IntakeConstants.INTAKE_SUB_READY) && !intake.IsBusy())
                    {
                        intake.setState(IntakeConstants.INTAKE);
                    }
                }
                setPathState(33);
                break;

            case 33:
                if(lastOuttakeState.equals(OuttakeConstants.SCORE_SPECIMEN) && !outtake.IsBusy())
                {

                }
                setPathState(34);
                break;


            case 100:
                telemetry.addLine("Done!");
                break;
        }

        lastOuttakeState = outtake.getState();
        lastIntakeState = intake.getState();
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        AutoPathUpdate();
    }
}
