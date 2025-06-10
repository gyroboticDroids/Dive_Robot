package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Outtake;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "6 + 0", group = "autonomous specimens", preselectTeleOp = "Master Tele-op")
public class SixSpec extends OpMode {
    private static final double SCORE_ZERO_POWER_ACCEL = 1.4;
    private static final double COLLECT_ZERO_POWER_ACCEL = 1.3;

    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;
    private Timer runTimer;
    private Intake intake;
    private Outtake outtake;
    private int pathState = -1, actionState = -1;
    private int loopCount;
    private boolean ons = false;

    double x = 0, y = 0, color = 2;

    private Path scorePreload, grabSpecimen1, scoreSpecimen1,
            intake1, intake2, intake3;

    //private PathChain[] grabSpecimenReady = new PathChain[4], scoreSpecimen = new PathChain[4];
    private final Path[] grabSpecimenReady = new Path[4], scoreSpecimen = new Path[4];

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_START), new Point(AutoConstants.SPECIMEN_SCORE_PRELOAD.getX(), AutoConstants.SPECIMEN_SCORE_PRELOAD.getY() - x)));
        scorePreload.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_START.getHeading(), AutoConstants.SPECIMEN_SCORE_PRELOAD.getHeading());
        scorePreload.setZeroPowerAccelerationMultiplier(3.5);

        intake1 = new Path(new BezierCurve(AutoConstants.SPECIMEN_SCORE_PRELOAD, AutoConstants.SPECIMEN_INTAKE_CONTROL_POINT, AutoConstants.SPECIMEN_INTAKE1));
        intake1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE_PRELOAD.getHeading(), AutoConstants.SPECIMEN_INTAKE1.getHeading());
        intake1.setZeroPowerAccelerationMultiplier(2.5);

        intake2 = new Path(new BezierLine(AutoConstants.SPECIMEN_INTAKE1, AutoConstants.SPECIMEN_INTAKE2));
        intake2.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_INTAKE1.getHeading(), AutoConstants.SPECIMEN_INTAKE2.getHeading());
        intake2.setZeroPowerAccelerationMultiplier(2);

        intake3 = new Path(new BezierLine(AutoConstants.SPECIMEN_INTAKE2, AutoConstants.SPECIMEN_INTAKE3));
        intake3.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_INTAKE2.getHeading(), AutoConstants.SPECIMEN_INTAKE3.getHeading());
        intake3.setZeroPowerAccelerationMultiplier(2);

        grabSpecimen1 = new Path(new BezierCurve(AutoConstants.SPECIMEN_INTAKE3, AutoConstants.SPECIMEN_GRAB_CONTROL_POINT, AutoConstants.SPECIMEN_GRAB));
        grabSpecimen1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_INTAKE3.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading(), 0.3);
        grabSpecimen1.setZeroPowerAccelerationMultiplier(1);

        scoreSpecimen1 = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB), AutoConstants.SPECIMEN_SCORING_CONTROL_POINT3, new Point(AutoConstants.SPECIMEN_SCORE1)));
        scoreSpecimen1.setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE1.getHeading());
        scoreSpecimen1.setZeroPowerAccelerationMultiplier(SCORE_ZERO_POWER_ACCEL);

        for (int i = 0; i < 4; i++) {
/*            grabSpecimenReady[i] = follower.pathBuilder()
                    .addPath(new BezierCurve(new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + (i + 1) * 1.2),
                            AutoConstants.SPECIMEN_SCORING_CONTROL_POINT1, new Point(AutoConstants.SPECIMEN_GRAB)))
                    .setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading())
                    .setZeroPowerAccelerationMultiplier(COLLECT_ZERO_POWER_ACCEL)
                    .build();

            scoreSpecimen[i] = follower.pathBuilder()
                    .addPath(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB), AutoConstants.SPECIMEN_SCORING_CONTROL_POINT3,
                            new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + (i + 1) * 1.2)))
                    .setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading())
                    .setZeroPowerAccelerationMultiplier(SCORE_ZERO_POWER_ACCEL)
                    .build();*/

            grabSpecimenReady[i] = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + (i + 1) * 0),
                            AutoConstants.SPECIMEN_SCORING_CONTROL_POINT2, new Point(AutoConstants.SPECIMEN_GRAB)));
            grabSpecimenReady[i].setLinearHeadingInterpolation(AutoConstants.SPECIMEN_SCORE.getHeading(), AutoConstants.SPECIMEN_GRAB.getHeading());
            grabSpecimenReady[i].setZeroPowerAccelerationMultiplier(COLLECT_ZERO_POWER_ACCEL);

            scoreSpecimen[i] = new Path(new BezierCurve(new Point(AutoConstants.SPECIMEN_GRAB), AutoConstants.SPECIMEN_SCORING_CONTROL_POINT3,
                            new Point(AutoConstants.SPECIMEN_SCORE.getX(), AutoConstants.SPECIMEN_SCORE.getY() + (i + 1) * 0)));
            scoreSpecimen[i].setLinearHeadingInterpolation(AutoConstants.SPECIMEN_GRAB.getHeading(), AutoConstants.SPECIMEN_SCORE.getHeading());
            scoreSpecimen[i].setZeroPowerAccelerationMultiplier(SCORE_ZERO_POWER_ACCEL);
        }
    }

    public void autonomousPathUpdate() {
        boolean robotInPos = follower.getCurrentTValue() >= 0.97;

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setActionState(7);
                setPathState(1);
                break;

            case 1:
                if(robotInPos) {
                    if(actionState == -1) {
                        setActionState(8);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if(actionState == -1) {
                    if(ons) {
                        follower.followPath(intake1);
                        ons = false;
                    } else if(follower.getCurrentTValue() > 0.5) {
                        setActionState(5);
                        setPathState(3);
                    }
                }
                break;


            case 3:
                if(robotInPos || !ons) {
                    if(actionState == -1) {
                        if(ons) {
                            setActionState(2);
                            ons = false;
                        } else {
                            follower.followPath(intake2);
                            setActionState(5);
                            setPathState(4);
                        }
                    }
                }
                break;

            case 4:
                if(robotInPos || !ons) {
                    if(actionState == -1) {
                        if(ons) {
                            setActionState(2);
                            ons = false;
                        } else {
                            follower.followPath(intake3);
                            setActionState(5);
                            setPathState(5);
                        }
                    }
                }
                break;

            case 5:
                if(robotInPos || !ons) {
                    if(actionState == -1) {
                        if(ons) {
                            setActionState(2);
                            ons = false;
                        } else {
                            setActionState(5);
                            setPathState(6);
                        }
                    }
                }
                break;

            case 6:
                if(actionState == -1) {
                    if(ons) {
                        pathTimer.resetTimer();
                        ons = false;
                    } else if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                        follower.followPath(grabSpecimen1);
                        setPathState(7);
                    }
                }
                break;


            case 7:
                if(robotInPos || !ons) {
                    if(actionState == -1 || pathTimer.getElapsedTimeSeconds() > 0.15) {
                        if(ons){
                            setActionState(0);
                            ons = false;
                            pathTimer.resetTimer();
                        }
                        else {
                            follower.followPath(scoreSpecimen1);

                            setPathState(8);
                        }
                    }
                }
                break;

            case 8:
                if(robotInPos || pathTimer.getElapsedTimeSeconds() > 3.5) {
                    if(actionState == -1) {
                        if (ons) {
                            setActionState(1);
                            ons = false;
                        }
                        else {
                            follower.followPath(grabSpecimenReady[4 - loopCount]);

                            setActionState(13);
                            setPathState(9);
                        }
                    }
                }
                break;

            case 9:
                if(robotInPos || !ons) {
                    if(actionState == -1 || pathTimer.getElapsedTimeSeconds() > 0.1) {
                        if(ons){
                            setActionState(0);
                            ons = false;
                            pathTimer.resetTimer();
                        }
                        else {
                            follower.followPath(scoreSpecimen[4 - loopCount]);

                            loopCount--;
                            if(loopCount > 0) {
                                setPathState(8);
                            } else {
                                setPathState(10);
                            }
                        }
                    }
                }
                break;

            case 10:
                if(robotInPos) {
                    if(actionState == -1) {
                        if (ons) {
                            setActionState(1);
                            ons = false;
                        }
                        else {
                            follower.followPath(grabSpecimenReady[3]);
                            setActionState(13);
                            setPathState(-1);
                        }
                    }
                }
                break;
        }
        telemetry.addData("robot in pos", robotInPos);
    }

    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                if(actionTimer.getElapsedTimeSeconds() > 0) {
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
                if(!intake.isBusy()) {
                    intake.setState(IntakeConstants.INTAKE);
                    intake.setHorizontalPosition(IntakeConstants.SLIDES_MAX - 300);
                    setActionState(3);
                }
                break;

            case 3:
                if(intake.getSampleColor() == color || intake.isSlidesAtSetpoint()) {
                    intake.setState(IntakeConstants.TRANSFER_FAST);
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    setActionState(15);
                }
                break;

            case 5:
                if(!intake.isBusy() && !outtake.isBusy()) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE);
                    setActionState(6);
                }
                break;

            case 6:
                if(!outtake.isBusy()) {
                    if(pathState <= 4) {
                        intake.setState(IntakeConstants.INTAKE_SUB_READY);
                        intake.setHorizontalPosition(IntakeConstants.SLIDES_OUT + 300);
                    }
                    outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
                    setActionState(-1);
                }
                break;


            case 7:
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_PRELOAD_READY);
                intake.setState(IntakeConstants.INTAKE_SUB_READY);
                intake.setHorizontalPosition(y * IntakeConstants.SLIDES_TICKS_PER_INCH + IntakeConstants.SLIDES_OUT);
                setActionState(14);
                break;

            case 8:
                outtake.setState(OuttakeConstants.SCORE_SPECIMEN_PRELOAD);
                setActionState(9);
                break;

            case 9:
                if(!outtake.isBusy() && !outtake.getState().equals(OuttakeConstants.TRANSFER_INTAKE_READY)) {
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                }

                if(actionTimer.getElapsedTimeSeconds() > 0.1 && !intake.getState().equals(IntakeConstants.INTAKE)) {
                    intake.setState(IntakeConstants.INTAKE);
                }

                if(actionTimer.getElapsedTimeSeconds() < 1.3 && intake.getSampleColor() != color) {
                    if(actionTimer.getElapsedTimeSeconds() > 0.3) {
                        intake.horizontalSlidesManual(10);
                    }
                } else {
                    intake.setState((intake.getSampleColor() == color)? IntakeConstants.TRANSFER : IntakeConstants.TRANSFER_REJECT);
                    setActionState(16);
                }
                break;

            case 13:
                if(follower.getPose().getX() < 35) {
                    outtake.setState(OuttakeConstants.GRAB_SPECIMEN_READY);
                    setActionState(14);
                }
                break;

            case 14:
                if(!outtake.isBusy()) {
                    setActionState(-1);
                }
                break;

            case 15:
                if(!intake.isBusy()) {
                    setActionState(-1);
                }
                break;

            case 16:
                if(intake.getHorizontalSlidePos() < IntakeConstants.SLIDES_OUT) {
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

    @Override
    public void init()
    {
        TransferConstants.resetConstants();

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        pathTimer = new Timer();
        actionTimer = new Timer();
        runTimer = new Timer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(AutoConstants.SPECIMEN_START);

        intake.setState(IntakeConstants.START);
        outtake.setState(OuttakeConstants.SPEC_PRELOAD_START);
    }

    boolean init = false;

    boolean g1Dpad = false;

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

        if(!g1Dpad) {
            x += ((gamepad1.dpad_right) ? 1 : -((gamepad1.dpad_left) ? 1 : 0)) / 2.0;
            y += ((gamepad1.dpad_up) ? 1 : -((gamepad1.dpad_down) ? 1 : 0)) / 2.0;
        }

        if (gamepad1.circle) {
            color = 2;
        } else if (gamepad1.square) {
            color = 3;
        }

        x = MathFunctions.clamp(x, -7, 7);
        y = MathFunctions.clamp(y, 0, Math.floor((IntakeConstants.SLIDES_MAX - IntakeConstants.SLIDES_OUT) / IntakeConstants.SLIDES_TICKS_PER_INCH));

        telemetry.addLine("sample x: " + x + ", y: " + y + ", color: " + ((color == 2) ? "RED" : "BLUE"));

        g1Dpad = gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right;

        if(init) {
            telemetry.addLine("initialized");
        } else {
            telemetry.addLine("NOT INITIALIZED");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        buildPaths();

        pathTimer.resetTimer();
        actionTimer.resetTimer();
        runTimer.resetTimer();
        intake.setIntakeWheelsKeepSpinning(true);
        intake.setState(IntakeConstants.START);

        loopCount = 4;
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
        telemetry.addData("parametric end", follower.atParametricEnd());
        telemetry.addData("t value", follower.getCurrentTValue());
        telemetry.addData("path state", pathState);
        telemetry.addData("action state", actionState);
        telemetry.addData("intake state", intake.getState());
        telemetry.addData("outtake state", outtake.getState());
        telemetry.addData("hori slide pos", intake.getHorizontalSlidePos());
        telemetry.addData("hori slide setpoint", intake.getHorizontalPosition());
        telemetry.addData("intake is busy", intake.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
