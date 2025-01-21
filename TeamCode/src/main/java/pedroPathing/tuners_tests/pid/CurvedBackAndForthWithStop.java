package pedroPathing.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is the CurvedBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * forward and to the left. On reaching the end of the forward Path, the robot runs the backward
 * Path the same distance back to the start. Rinse and repeat! This is good for testing a variety
 * of Vectors, like the drive Vector, the translational Vector, the heading Vector, and the
 * centripetal Vector. Remember to test your tunings on StraightBackAndForth as well, since tunings
 * that work well for curves might have issues going in straight lines.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@Config
@Autonomous (name = "Curved Back And Forth With Stop", group = "PIDF Testing")
public class CurvedBackAndForthWithStop extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 20;

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

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(12,0, Point.CARTESIAN)));
        forwards.setLinearHeadingInterpolation(0,Math.toRadians(90));
        backwards = new Path(new BezierLine(new Point(12,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setLinearHeadingInterpolation(Math.toRadians(90),0);

        //backwards.setReversed(true);

        follower.followPath(forwards, true);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a curve going " + 12 + " inches"
                            + " to the left and the same number of inches forward. The robot will go"
                            + "forward and backward continuously along the path. Make sure you have"
                            + "enough room.");
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
                    follower.followPath(backwards,true);
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
