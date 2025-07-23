package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Outtake;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Disabled
@Autonomous(name = "auto training", group = "training", preselectTeleOp = "Master Tele-op")
public class AutoTrainingCourse extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;
    private Intake intake;
    private Outtake outtake;
    private int pathState = -1, actionState = -1;


    private Path scorePreload, grabSpecimen1;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_START), new Point(AutoConstants.SPECIMEN_SCORE_PRELOAD)));
        scorePreload.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_START.getHeading(), AutoConstants.SPECIMEN_SCORE_PRELOAD.getHeading());
        scorePreload.setZeroPowerAccelerationMultiplier(4);

        grabSpecimen1 = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE_PRELOAD), new Point(AutoConstants.SPECIMEN_GRAB)));
        grabSpecimen1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE_PRELOAD.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading());
        grabSpecimen1.setZeroPowerAccelerationMultiplier(1.5);
    }

    public void autonomousPathUpdate() {
        boolean robotInPos = follower.getCurrentTValue() >= 0.97;

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setActionState(0);
                setPathState(1);
                break;

            case 1:
                if(robotInPos && actionState == -1) {
                    follower.followPath(grabSpecimen1);
                    setActionState(1);
                    setPathState(2);
                }
                break;

            case 2:
                if(robotInPos && actionState == -1) {
                    //TODO: Make robot grab specimen and drive to bar
                }
                break;

            case 3:
                if(robotInPos && actionState == -1) {
                    //TODO: Make robot score specimen
                }
                break;

            //TODO: Add more states
        }
        telemetry.addData("robot in pos", robotInPos);
    }

    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_PRELOAD_READY);
                setActionState(14);
                break;

            case 1:
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_PRELOAD);
                setActionState(2);
                break;

            case 2:
                if(!outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
                    setActionState(14);
                }
                break;

            case 3:
                //TODO: Make robot grab specimen
                break;

            case 4:
                //TODO: Make robot score specimen
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
        pathTimer.resetTimer();
    }

    public void setActionState(int aState) {
        actionState = aState;
        actionTimer.resetTimer();
    }

    @Override
    public void init()
    {
        TransferConstants.resetConstants();

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        pathTimer = new Timer();
        actionTimer = new Timer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(AutoConstants.SPECIMEN_START);
        buildPaths();

        intake.setState(IntakeConstants.START);
        outtake.setState(OuttakeConstants.SPEC_PRELOAD_START);
    }

    boolean init = false;

    @Override
    public void init_loop()
    {
        //Resets intake pos
        intake.update();
        outtake.update();

        if(actionTimer.getElapsedTimeSeconds() > 4 && !(intake.getState().equals(IntakeConstants.RESET_POS) || intake.getState().equals(IntakeConstants.TRANSFER))){
            intake.setState(IntakeConstants.RESET_POS);
        }

        if (!intake.isBusy() && intake.getState().equals(IntakeConstants.RESET_POS))
        {
            intake.setState(IntakeConstants.TRANSFER);
            init = true;
        }

        if(init) {
            telemetry.addLine("initialized");
        } else {
            telemetry.addLine("NOT INITIALIZED");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        intake.setState(IntakeConstants.START);

        setPathState(0);
    }

    @Override
    public void stop() {
        TransferConstants.horiSlidePos = intake.getHorizontalSlidePos();
        TransferConstants.heading = Math.toDegrees(follower.getPose().getHeading());
        TransferConstants.endPose = follower.getPose();
    }

    @Override
    public void loop()
    {
        follower.update();
        outtake.update();
        intake.update();
        autonomousPathUpdate();
        autonomousActionUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("action state", actionState);
        telemetry.addData("intake state", intake.getState());
        telemetry.addData("outtake state", outtake.getState());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
