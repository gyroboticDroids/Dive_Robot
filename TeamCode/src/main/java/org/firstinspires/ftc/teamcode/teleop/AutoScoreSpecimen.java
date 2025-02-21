package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;

public class AutoScoreSpecimen {

    private int slideRangeSubtract = 0;
    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;
    private Intake intake;
    private Outtake outtake;
    private int pathState = -1;
    private int actionState = -1;
    private boolean onsIntakeOut = false;
    private boolean onsGrabSample = false;
    private boolean robotInPos;
    private Path currentPath;
    private double currentHeading;

    private Path scoreSpecimen, grabSpecimenReady, grabSpecimen;

    public void buildPaths()
    {
        grabSpecimen = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_GRAB_READY), new Point(AutoConstants.SPECIMEN_GRAB)));
        grabSpecimen.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB_READY.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading());
    }

    void autoscoreSpecimen(){
        boolean robotInPos = MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getX(), follower.getPose().getX(), 1) &&
                MathFunctions.roughlyEquals(currentPath.getLastControlPoint().getY(), follower.getPose().getY(), 1);

        switch (pathState) {

            case 2:
                if (robotInPos) {
                    if (onsIntakeOut) {
                        pathTimer.resetTimer();
                        onsIntakeOut = false;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        currentPath = grabSpecimen;
                        follower.followPath(currentPath, true);
                        setPathState(3);
                    }
                }
                break;

            case 3:
                if (robotInPos) {
                    if (actionState == -1 || actionTimer.getElapsedTimeSeconds() > 0.5) {
                        if (onsIntakeOut) {
                            setActionState(0);
                            onsIntakeOut = false;
                            actionTimer.resetTimer();
                        } else {
                            currentPath = scoreSpecimen;
                            follower.followPath(currentPath, true);
                            setPathState(4);
                        }
                    }
                }
                break;

            case 4:
                if (robotInPos) {
                    if (actionState == -1) {
                        if (onsIntakeOut) {
                            setActionState(1);
                            onsIntakeOut = false;
                        } else {
                            currentPath = grabSpecimenReady;
                            follower.followPath(currentPath, true);
                            setActionState(13);
                            setPathState(5);
                        }
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
                    if(actionTimer.getElapsedTimeSeconds() > 0.35) {
                        outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
                        setActionState(14);
                    }
                    break;

                case 13:
                    if(actionTimer.getElapsedTimeSeconds() > 0.25) {
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
        onsIntakeOut = true;
        pathTimer.resetTimer();
    }

    public void setActionState(int aState) {
        actionState = aState;
        onsGrabSample = true;
        actionTimer.resetTimer();
    }
}

