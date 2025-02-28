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
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;

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
    private boolean followPath = false;
    private Path currentPath;

    private Path scoreSample;

    public AutoScoreSample(HardwareMap hardwareMap) {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        follower.poseUpdater.setCurrentPoseWithOffset(TransferConstants.endPose);

        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);

        pathTimer = new Timer();
        actionTimer = new Timer();

        stopAuto();
    }

    private void autonomousPathUpdate() {
        if(scoreSample == null) {
            return;
        }

        boolean robotInPos = MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getX(), follower.getPose().getX(), 1) &&
                MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getY(), follower.getPose().getY(), 1);

        switch (pathState) {
            case 0:
                if(onsPath) {
                    setActionState(0);
                    onsPath = false;
                }

                if(intake.getState().equals(IntakeConstants.TRANSFER) && intake.getHorizontalPosition() == IntakeConstants.SLIDES_TRANSFER) {
                    currentPath = scoreSample;
                    follower.followPath(currentPath, true);
                    setPathState(1);
                }
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
                if(intake.getState().equals(IntakeConstants.TRANSFER) && (outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE_READY) || outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE))) {
                    setActionState(1);
                }
                else {
                    if(!(outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE_READY) || outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE)) && !outtake.isBusy()) {
                        outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    }

                    if(!intake.getState().equals(IntakeConstants.TRANSFER) && !intake.isBusy()) {
                        intake.setState(IntakeConstants.TRANSFER);
                    }
                }
                break;

            case 1:
                if (!outtake.isBusy() && !intake.isBusy() && !outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE)) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
                    setActionState(2);
                }
                break;

            case 2:
                if (!outtake.isBusy()) {
                    if(intake.getSampleColor() > 0) {
                        stopAuto();
                        return;
                    }

                    outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
                    setActionState(10);
                }

                break;

            case 10:
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

    public boolean startAuto() {
        Pose currentPos = follower.getPose();

        if(currentPos.getX() < 48) {
            scoreSample = new Path(new BezierCurve(new Point(currentPos), new Point(AutoConstants.SAMPLE_SCORE.getX(), currentPos.getY()), new Point(AutoConstants.SAMPLE_SCORE)));
            scoreSample.setLinearHeadingInterpolation(currentPos.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading());
            scoreSample.setZeroPowerAccelerationMultiplier(1.5);
        } else if (currentPos.getY() > 86) {
            scoreSample = new Path(new BezierCurve(new Point(currentPos), new Point(currentPos.getX(), AutoConstants.SAMPLE_SCORE.getY()), new Point(AutoConstants.SAMPLE_SCORE)));
            scoreSample.setLinearHeadingInterpolation(currentPos.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading());
            scoreSample.setZeroPowerAccelerationMultiplier(1.5);
        } else {
            return false;
        }

        follower.setStartingPose(currentPos);
        followPath = true;
        setPathState(0);
        return true;
    }

    public void stopAuto() {
        setPathState(-1);
        setActionState(-1);
        follower.breakFollowing();
        scoreSample = null;

        followPath = false;
    }

    public void resetPos(Pose pose) {
        follower.poseUpdater.setCurrentPoseWithOffset(pose);
    }

    public boolean isPathing() {
        return followPath;
    }

    public void update()
    {
        follower.update();

        if(!followPath) {
            return;
        }

        outtake.update();
        intake.update();
        autonomousPathUpdate();
        autonomousActionUpdate();
    }
}

