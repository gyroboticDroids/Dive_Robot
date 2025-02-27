package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class AutoScoreSample {

    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;
    private Intake intake;
    private Outtake outtake;
    private int pathState = -1;
    private int actionState = -1;
    private boolean onsPath = false;
    private boolean onsAction = false;
    private boolean robotInPos;
    private boolean followPath = false;
    private Path currentPath;
    private double currentHeading;

    private Path scoreSample;

    public AutoScoreSample(HardwareMap hardwareMap) {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);

        stopAuto();
    }

    private void autonomousPathUpdate() {
        boolean robotInPos = MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getX(), follower.getPose().getX(), 1) &&
                MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getY(), follower.getPose().getY(), 1);

        switch (pathState) {
            case 0:
                if(scoreSample == null) {
                    return;
                }

                currentPath = scoreSample;
                follower.followPath(currentPath, true);
                setActionState(0);
                setPathState(3);
                break;

            case 1:
                if (robotInPos) {
                    if (actionState == -1) {
                        stopAuto();
                    }
                }
                break;
        }
    }

    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
                setActionState(14);
                break;

            case 1:
                if (actionTimer.getElapsedTimeSeconds() > 0.35) {
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
                    setActionState(14);
                }
                break;

            case 13:
                if (actionTimer.getElapsedTimeSeconds() > 0.25) {
                    outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
                    setActionState(14);
                }

                break;

            case 14:
                if (!outtake.isBusy()) {
                    setActionState(-1);
                }
                break;
        }
    }


    public void setPathState(int pState) {
        pathState = pState;
        onsPath = true;
        pathTimer.resetTimer();
    }

    public void setActionState(int aState) {
        actionState = aState;
        onsAction = true;
        actionTimer.resetTimer();
    }

    public void startAuto(Pose currentPos) {
        setPathState(0);
        follower.setStartingPose(currentPos);

        scoreSample = new Path(new BezierCurve(new Point(currentPos), new Point(currentPos.getX(), AutoConstants.SAMPLE_SCORE.getY()), new Point(AutoConstants.SAMPLE_SCORE)));
        scoreSample.setLinearHeadingInterpolation(currentPos.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading());
        scoreSample.setZeroPowerAccelerationMultiplier(1.5);

        followPath = true;
    }

    public void stopAuto() {
        setPathState(-1);
        setActionState(-1);
        follower.breakFollowing();
        scoreSample = null;

        followPath = false;
    }

    public void update()
    {
        if(!followPath) {
            return;
        }

        follower.update();
        outtake.update();
        intake.update();
        autonomousPathUpdate();
        autonomousActionUpdate();
    }
}

