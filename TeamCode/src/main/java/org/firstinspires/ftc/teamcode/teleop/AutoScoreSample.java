package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
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
    private DashboardPoseTracker dashboardPoseTracker;
    private int pathState = -1;
    private int actionState = -1;
    private boolean onsPath = false;
    private boolean onsAction = false;
    private boolean followPath = false;
    private Path currentPath;

    private Path scoreSample;

    public AutoScoreSample(HardwareMap hardwareMap, Outtake out, Intake in) {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        follower.setStartingPose(TransferConstants.endPose);
        dashboardPoseTracker = new DashboardPoseTracker(follower.poseUpdater);

        outtake = out;
        intake = in;

        pathTimer = new Timer();
        actionTimer = new Timer();

        runAuto(false);
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
                    follower.followPath(currentPath, false);
                    setPathState(1);
                }
                break;

            case 1:
                if (robotInPos) {
                    if (actionState == -1) {
                        if(onsPath) {
                            setActionState(5);
                            onsPath = false;
                        } else {
                            runAuto(false);
                        }
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
                if (!outtake.isBusy() && !intake.isBusy()) {
                    if(!outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE)) {
                        outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
                    }
                    setActionState(2);
                }
                break;

            case 2:
                if (!outtake.isBusy()) {
                    if(intake.getSampleColor() > 0 && outtake.isSlidesAtSetpoint()) {
                        setActionState(15);
                        return;
                    }

                    outtake.setState(OuttakeConstants.SCORE_SAMPLE_READY_HIGH);
                    intake.setState(IntakeConstants.INTAKE_SUB_READY);
                    setActionState(10);
                }

                break;

            case 5:
                if (!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.SCORE_SAMPLE);
                    setActionState(-1);
                }

                break;

            case 10:
                if (!outtake.isBusy()) {
                    setActionState(-1);
                }
                break;

            case 15:
                if(intake.getState().equals(IntakeConstants.INTAKE_SUB_READY) && outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE_READY)) {
                    setActionState(0);
                }
                else {
                    if(!outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE_READY) && !outtake.isBusy()) {
                        outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    }

                    if(!intake.getState().equals(IntakeConstants.INTAKE_SUB_READY) && !intake.isBusy()) {
                        intake.setState(IntakeConstants.INTAKE_SUB_READY);
                    }
                }
                break;
        }

        outtake.update();
        intake.update();
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

    public void runAuto(boolean run) {
        followPath = run;
    }

    public void resetPos(Pose pose) {
        follower.setCurrentPoseWithOffset(pose);
    }

    public Pose currentPose() {
        return follower.getPose();
    }

    public boolean isPathing() {
        return !(!followPath && !prevFollow);
    }

    boolean prevFollow = false;

    public void update()
    {
        dashboardPoseTracker.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(follower.getPose().getAsFTCStandardCoordinates(), "#4CAF50");
        Drawing.sendPacket();

        follower.update();

        if(followPath && !prevFollow) {
            Pose currentPos = follower.getPose();

            if(currentPos.getX() < 48) {
                scoreSample = new Path(new BezierCurve(new Point(currentPos), new Point(AutoConstants.SAMPLE_SCORE.getX(), currentPos.getY()), new Point(AutoConstants.SAMPLE_SCORE)));
                scoreSample.setLinearHeadingInterpolation(currentPos.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading(), 0.7);
                scoreSample.setZeroPowerAccelerationMultiplier(2);
            } else if (currentPos.getY() > 86) {
                scoreSample = new Path(new BezierCurve(new Point(currentPos), new Point(currentPos.getX() + 2, AutoConstants.SAMPLE_SCORE.getY() - 25), new Point(AutoConstants.SAMPLE_SCORE)));
                scoreSample.setLinearHeadingInterpolation(currentPos.getHeading(), AutoConstants.SAMPLE_SCORE.getHeading(), 0.7);
                scoreSample.setZeroPowerAccelerationMultiplier(2);
            } else {
                runAuto(false);
            }

            currentPath = scoreSample;

            setPathState(0);
            setActionState(-1);
        }
        else if(!followPath && prevFollow) {
            follower.breakFollowing();

            scoreSample = null;

            setPathState(-1);
            setActionState(-1);
        }

        if(followPath) {
            autonomousPathUpdate();
            autonomousActionUpdate();
        }

        prevFollow = followPath;
    }
}

