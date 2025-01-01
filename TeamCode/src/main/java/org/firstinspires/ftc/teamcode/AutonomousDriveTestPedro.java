package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class AutonomousDriveTestPedro extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private Pose startPose = new Pose(0, 0, Math.toRadians(180));
    private Pose point1 = new Pose(55, -55,Math.toRadians(180));
    private Pose point2 = new Pose(5.2, -103.9,Math.toRadians(180));
    private Pose turn1 = new Pose(5.1, -103.9,Math.toRadians(0));
    private Pose point3 = new Pose(turn1.getX() + 50, -103.9,Math.toRadians(180));
    private Pose point4 = new Pose(turn1.getX() + 50, 0,Math.toRadians(180));

    private PathChain driveToBox;
    private Path turn, driveBack, driveRight, driveToStart;

    public void BuildPaths()
    {
        driveToBox = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(point1.getX(), startPose.getY()), new Point(point1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), point1.getHeading())
                .addPath(new BezierCurve(new Point(point1), new Point(point1.getX(), point2.getY()), new Point(point2)))
                .setLinearHeadingInterpolation(point1.getHeading(), point2.getHeading())
                .build();

        turn = new Path(new BezierLine(new Point(point2), new Point(turn1)));
        turn.setLinearHeadingInterpolation(point2.getHeading(), turn1.getHeading());

        driveBack = new Path(new BezierLine(new Point(point2), new Point(point3)));
        driveBack.setLinearHeadingInterpolation(point2.getHeading(), point3.getHeading());

        driveRight = new Path(new BezierLine(new Point(point3), new Point(point4)));
        driveRight.setLinearHeadingInterpolation(point3.getHeading(), point4.getHeading());

        driveToStart = new Path(new BezierLine(new Point(point4), new Point(startPose)));
        driveToStart.setLinearHeadingInterpolation(point4.getHeading(), startPose.getHeading());

    }

    public void AutoPathUpdate()
    {
        switch (pathState)
        {
            case 10:
                follower.followPath(driveToBox, true);
                setPathState(11);
                break;

            case 11:
                if(!follower.isBusy())
                {
                    setPathState(20);
                }
                break;

            case 20:
                follower.followPath(turn, true);
                setPathState(21);
                break;

            case 21:
                if(!follower.isBusy())
                {
                    setPathState(22);
                }
                break;

            case 22:
                if(pathTimer.getElapsedTimeSeconds() > 1)
                {
                    setPathState(30);
                }
                break;

            case 30:
                follower.followPath(driveBack, true);
                setPathState(31);
                break;

            case 31:
                if(!follower.isBusy())
                {
                    setPathState(40);
                }
                break;

            case 40:
                follower.followPath(driveRight, true);
                setPathState(41);
                break;

            case 41:
                if(!follower.isBusy())
                {
                    setPathState(50);
                }
                break;

            case 50:
                follower.followPath(driveToStart, true);
                setPathState(51);
                break;

            case 51:
                break;
        }
    }

    @Override
    public void loop() {
        // These loop the actions and movement of the robot
        follower.update();
        AutoPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("follower is busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        BuildPaths();

        setPathState(10);
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        AutoPathUpdate();
    }
}
