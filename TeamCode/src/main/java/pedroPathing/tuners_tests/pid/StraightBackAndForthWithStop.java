package pedroPathing.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
//@Disabled
@Config
@Autonomous (name = "Straight Back And Forth With Stop", group = "PIDF Tuning")
public class StraightBackAndForthWithStop extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;
    private Timer timer;

    private Path forwards;
    private Path backwards;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(AutoConstants.SPECIMEN_START);

        forwards = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_START), new Point(AutoConstants.SPECIMEN_SCORE)));
        forwards.setLinearHeadingInterpolation(0,0);
        forwards.setZeroPowerAccelerationMultiplier(2.5);
        backwards = new Path(new BezierLine(new Point(AutoConstants.SPECIMEN_SCORE), new Point(AutoConstants.SPECIMEN_START)));
        backwards.setLinearHeadingInterpolation(0,0);
        backwards.setZeroPowerAccelerationMultiplier(2.5);

        follower.followPath(forwards, true);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                            + " inches forward. The robot will go forward and backward continuously"
                            + " along the path. Make sure you have enough room.");
        telemetryA.update();

        timer = new Timer();
        timer.resetTimer();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if (forward) {
                if(timer.getElapsedTimeSeconds() > 15) {
                    forward = false;
                    follower.followPath(backwards, true);
                    timer.resetTimer();
                }
            } else {
                if(timer.getElapsedTimeSeconds() > 15) {
                    forward = true;
                    follower.followPath(forwards, true);
                    timer.resetTimer();
                }
            }
        }

        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
