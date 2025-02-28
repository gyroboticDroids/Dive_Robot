package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class AutoScoreSpecimen {

    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;
    private Intake intake;
    private Outtake outtake;
    private int pathState = -1;
    private int actionState = -1;
    private boolean ons = false;
    private boolean followPath = false;
    private boolean robotInPos;
    private Path currentPath;
    private double currentHeading;


    private Path scoreSpecimen, grabSpecimenReady;

    public void buildPaths()
    {
        scoreSpecimen = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_GRAB), new Point(42.875, 70)));
        scoreSpecimen.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());

        grabSpecimenReady = new Path(new BezierCurve(new Point(42.875, 70), new Point(AutoConstants.SPECIMEN_SCORE)));
        grabSpecimenReady.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
    }

    public AutoScoreSpecimen(HardwareMap hardwareMap) {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);

        pathTimer = new Timer();
        actionTimer = new Timer();
    }

    public void autonomousPathUpdate(){
        boolean robotInPos = MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getX(), follower.getPose().getX(), 1) &&
                MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getY(), follower.getPose().getY(), 1);

        switch (pathState) {

            case 0:
                if(robotInPos) {
                    if(actionState == -1 || pathTimer.getElapsedTimeSeconds() > 0.35) {
                        if(ons){
                            setActionState(0);
                            ons = false;
                            pathTimer.resetTimer();
                        }
                        else {
                            scoreSpecimen = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_GRAB), new Point(42.875, 70)));
                            scoreSpecimen.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
                            scoreSpecimen.setZeroPowerAccelerationMultiplier(1.1);
                            follower.followPath(currentPath, true);
                            currentPath = grabSpecimenReady;
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
                            grabSpecimenReady = new Path(new BezierCurve(new Point(42.875, 70), new Point(AutoConstants.SPECIMEN_SCORE)));
                            grabSpecimenReady.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
                            grabSpecimenReady.setZeroPowerAccelerationMultiplier(1.2);
                            follower.followPath(currentPath, true);
                            currentPath = grabSpecimenReady;
                            followPath = true;
                            setActionState(2);
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
                    if(actionTimer.getElapsedTimeSeconds() > 0.15) {
                        outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
                        setActionState(14);
                    }
                    break;

                case 2:
                    if(actionTimer.getElapsedTimeSeconds() > 0) {
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
        ons = true;
        actionTimer.resetTimer();
    }

    public void stopAuto() {
        setPathState(-1);
        setActionState(-1);
        follower.breakFollowing();
        scoreSpecimen = null;

        followPath = false;
    }

    public void update()
    {
        follower.update();
        outtake.update();
        intake.update();
        autonomousPathUpdate();
        autonomousActionUpdate();
    }
}

