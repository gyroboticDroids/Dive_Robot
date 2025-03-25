package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class AutoScoreSpecimen {

    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;
    private Outtake outtake;
    private HardwareMap hMap;
    private int pathState = -2;
    private int actionState = -1;
    private boolean ons = false;
    private boolean followPath = false;
    private Path currentPath;
    private int yIncrement = 1;

    private double offset = 0;

    private Path scoreSpecimen, grabSpecimenReady;

    public AutoScoreSpecimen(HardwareMap hardwareMap, Outtake out) {
        outtake = out;

        hMap = hardwareMap;

        pathTimer = new Timer();
        actionTimer = new Timer();
    }

    public void autonomousPathUpdate(){
        boolean robotInPos = MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getX(), follower.getPose().getX(), 1) &&
                MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getY(), follower.getPose().getY(), 1);

        switch (pathState) {
            case -1:
                if(ons) {
                    setActionState(3);
                    ons = false;
                } else if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    scoreSpecimen = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB.getX() + offset, AutoConstants.SPECIMEN_GRAB.getY()),
                            new Point(AutoConstants.SPECIMEN_SCORING_CONTROL_POINT3.getX() + offset, AutoConstants.SPECIMEN_SCORING_CONTROL_POINT3.getY() + 1.25 * yIncrement),
                            new Point(AutoConstants.SPECIMEN_SCORE.getX() + offset, AutoConstants.SPECIMEN_SCORE.getY() + 1.25 * yIncrement)));
                    scoreSpecimen.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
                    scoreSpecimen.setZeroPowerAccelerationMultiplier(1.1);

                    currentPath = scoreSpecimen;

                    follower.followPath(currentPath, true);
                    followPath = true;
                    setPathState(1);
                }
                break;

            case 0:
                if(robotInPos) {
                    if(actionState == -1 || pathTimer.getElapsedTimeSeconds() > 0.3) {
                        if(ons){
                            setActionState(0);
                            ons = false;
                            pathTimer.resetTimer();
                        }
                        else {
                            scoreSpecimen = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB.getX() + offset, AutoConstants.SPECIMEN_GRAB.getY()),
                                    new Point(AutoConstants.SPECIMEN_SCORING_CONTROL_POINT3.getX() + offset, AutoConstants.SPECIMEN_SCORING_CONTROL_POINT3.getY() + 1.25 * yIncrement),
                                    new Point(AutoConstants.SPECIMEN_SCORE.getX() + offset, AutoConstants.SPECIMEN_SCORE.getY() + 1.25 * yIncrement)));
                            scoreSpecimen.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
                            scoreSpecimen.setZeroPowerAccelerationMultiplier(1.1);

                            currentPath = scoreSpecimen;

                            follower.followPath(currentPath, true);
                            followPath = true;
                            setPathState(1);
                        }
                    }
                }
                break;

            case 1:
                if(robotInPos) {
                    if(actionState == -1) {
                        if (ons) {
                            setActionState(1);
                            ons = false;
                        }
                        else {
                            grabSpecimenReady = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_SCORE.getX() + offset, AutoConstants.SPECIMEN_SCORE.getY() + 1.25 * yIncrement),
                                    new Point(AutoConstants.SPECIMEN_SCORING_CONTROL_POINT1.getX() + offset, AutoConstants.SPECIMEN_SCORING_CONTROL_POINT1.getY()),
                                    new Point(AutoConstants.SPECIMEN_SCORING_CONTROL_POINT2.getX() + offset, AutoConstants.SPECIMEN_SCORING_CONTROL_POINT2.getY()),
                                    new Point(AutoConstants.SPECIMEN_GRAB.getX() + offset, AutoConstants.SPECIMEN_GRAB.getY())));
                            grabSpecimenReady.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading());
                            grabSpecimenReady.setZeroPowerAccelerationMultiplier(1.1);

                            currentPath = grabSpecimenReady;

                            follower.followPath(currentPath, true);
                            followPath = true;
                            setActionState(13);
                            yIncrement++;
                            setPathState(0);
                        }
                    }
                }
                break;
            }
        }

        public void autonomousActionUpdate() {
            switch (actionState) {
                case 0:
                    if(actionTimer.getElapsedTimeSeconds() > 0.15) {
                        outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
                        setActionState(14);
                    }
                    break;

                case 1:
                    if(actionTimer.getElapsedTimeSeconds() > 0) {
                        outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
                        setActionState(14);
                    }
                    break;

                case 2:
                    if(actionTimer.getElapsedTimeSeconds() > 0.5) {
                        outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                        setActionState(14);
                    }
                    break;

                case 3:
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
                    setActionState(14);
                    break;

                case 13:
                    if(follower.getPose().getX() < 37) {
                        outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
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
        ons = true;
        pathTimer.resetTimer();
    }

    public void setActionState(int aState) {
        actionState = aState;
        actionTimer.resetTimer();
    }

    public void startAuto() {
        setPathState(-1);
        setActionState(-1);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hMap);
        follower.setStartingPose(AutoConstants.SPECIMEN_GRAB);

        grabSpecimenReady = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_SCORE), AutoConstants.SPECIMEN_SCORING_CONTROL_POINT1, AutoConstants.SPECIMEN_SCORING_CONTROL_POINT2, new Point(AutoConstants.SPECIMEN_GRAB)));
        grabSpecimenReady.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading());
        grabSpecimenReady.setZeroPowerAccelerationMultiplier(1.1);

        currentPath = grabSpecimenReady;

        followPath = true;
    }

    public void stopAuto() {
        setPathState(-2);
        setActionState(-1);
        grabSpecimenReady = null;
        scoreSpecimen = null;
        follower = null;

        followPath = false;
    }

    public boolean isPathing() {
        return followPath;
    }

    public int pathState() {
        return pathState;
    }

    public int actionState() {
        return actionState;
    }

    boolean prevButton = false;

    public void increment(Gamepad gpad) {
        if(!prevButton) {
            if(gpad.a){
                offset += 0.1;
            } else if (gpad.y) {
                offset -= 0.1;
            }
        }

        prevButton = gpad.a || gpad.y;
    }

    public void update()
    {
        if(!followPath) {
            return;
        }

        autonomousPathUpdate();
        autonomousActionUpdate();
        follower.update();
        outtake.update();
    }
}

