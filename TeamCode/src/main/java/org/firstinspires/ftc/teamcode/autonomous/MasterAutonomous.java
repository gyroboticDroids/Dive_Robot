package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Outtake;

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

    int startPos = 0;
    boolean isPressed;

    //Poses
    private Pose start;
    private Pose specimenLeft = new Pose(36, 11.25, 0);
    private Pose specimenRight = new Pose(36, -11.25, 0);
    private Pose basketSample1 = new Pose(7, -50, Math.toRadians(25));
    private Pose basketSample2 = new Pose(8, -51, Math.toRadians(13));
    private Pose sample3 = new Pose(12, -45, Math.toRadians(-45));
    private Pose basketSample3 = new Pose(7, -48, Math.toRadians(45));
    private Pose touchBar = new Pose(72, -12, Math.toRadians(0));

    //Paths
    private Path toLeftOfBar, toBasket1, toBasket2, toSample3, toBasket3, toBar;

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
        else
        {
            isPressed = false;
        }

        startPos = (int) MathFunctions.clamp(startPos, 0, 3);

        telemetry.addData("Start Position Index: ", startPos);
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
        AutoPathUpdate();
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

        setPathState(0);
    }

    private void AutoPathUpdate()
    {
        switch (pathState)
        {
            case 0:
                if(scoreBoth)
                    setPathState(1);
                if(scoreSpecimens)
                    setPathState(20);
                if(scoreSamples)
                    setPathState(10);
                break;

            case 1:
                follower.followPath(toLeftOfBar, true);
                setPathState(2);
                break;

            case 2:
                outtake.SetState("score specimen ready high");

                if(!follower.isBusy())
                {
                    outtake.SetState("score specimen");
                    setPathState(3);
                }
                break;

            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 1)
                {
                    setPathState(10);
                }
                break;

            case 10:
                follower.followPath(toBasket1);
                setPathState(11);
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        AutoPathUpdate();
    }
}
