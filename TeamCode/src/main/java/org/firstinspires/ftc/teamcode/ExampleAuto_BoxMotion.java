package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

/**
 * This is an example auto that showcases movement and control of three servos autonomously.
 * It is able to detect the team element using a huskylens and then use that information to go to the correct spike mark and backdrop position.
 * There are examples of different ways to build paths.
 * A custom action system have been created that can be based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

@Autonomous(name = "Example Box Motion", group = "Examples")
public class ExampleAuto_BoxMotion extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState, actionState, clawState;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Centerstage, this would be blue far side/red human player station.)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y. **/
    //Start Pose
    private Pose startPose = new Pose(0, 0, Math.toRadians(0));
    //Positions
    private Pose corner2 = new Pose(20, 0, Math.toRadians(0));
    private Pose corner3 = new Pose(20, 20, Math.toRadians(0));
    private Pose corner4 = new Pose(0, 20, Math.toRadians(0));

    // Poses and Paths for Purple and Yellow
    //private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;
    private Path line1, line2, line3, line4;
    private PathChain boxPathChain;

    // White Stack Cycle Poses + Path Chains
    //private Pose TopTruss = new Pose(28, 84, Math.toRadians(270));
    //private Pose BottomTruss = new Pose(28, 36, Math.toRadians(270));
    //private Pose Stack = new Pose(46, 11.5, Math.toRadians(270));
    //private PathChain cycleStackTo, cycleStackBack, cycleStackToBezier;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        line1 = new Path(new BezierLine(new Point(startPose), new Point(corner2)));
        line1.setLinearHeadingInterpolation(startPose.getHeading(), corner2.getHeading());
        line1.setPathEndTimeoutConstraint(0);

        line2 = new Path(new BezierLine(new Point(corner2), new Point(corner3)));
        line2.setLinearHeadingInterpolation(corner2.getHeading(), corner3.getHeading());
        line2.setPathEndTimeoutConstraint(0);

        line3 = new Path(new BezierLine(new Point(corner3), new Point(corner4)));
        line3.setLinearHeadingInterpolation(corner3.getHeading(), corner4.getHeading());
        line3.setPathEndTimeoutConstraint(0);

        line4 = new Path(new BezierLine(new Point(corner4), new Point(startPose)));
        line4.setLinearHeadingInterpolation(corner4.getHeading(), startPose.getHeading());
        line4.setPathEndTimeoutConstraint(0);

        /** There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require > 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://www.desmos.com/calculator/3so1zx0hcd).
         *    * BezierLines are straight, and require 2 points. There are the start and end points. **/

        /** This is a path chain, defined on line 66
         * It, well, chains multiple paths together. Here we use a constant heading from the board to the stack.
         * On line 97, we set the Linear Interpolation,
         * which means that Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path */

         boxPathChain = follower.pathBuilder()
                 .addPath(new BezierLine(new Point(startPose), new Point(corner2)))
                 .setLinearHeadingInterpolation(startPose.getHeading(), corner2.getHeading())
                 .addPath(new BezierLine(new Point(corner2), new Point(corner3)))
                 .setLinearHeadingInterpolation(corner2.getHeading(), corner3.getHeading())
                 .addPath(new BezierLine(new Point(corner3), new Point(corner4)))
                 .setLinearHeadingInterpolation(corner3.getHeading(), corner4.getHeading())
                 .addPath(new BezierLine(new Point(corner4), new Point(startPose)))
                 .setLinearHeadingInterpolation(corner4.getHeading(), startPose.getHeading())
                 .build();
    }
    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() function on line 193)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                setActionState(0);
                follower.followPath(line1, true);
                setPathState(12);
                break;
            case 11:
                if(!follower.isBusy())
                {
                    follower.holdPoint(corner2);
                    setPathState(12);
                }
                break;
            case 12:
                if(gamepad1.a && !follower.isBusy())
                {
                    setPathState(20);
                }
                break;
            case 20:
                follower.followPath(line2, true);
                setPathState(22);
                break;
            case 21:
                if(!follower.isBusy())
                {
                    follower.holdPoint(corner3);
                    setPathState(22);
                }
                break;
            case 22:
                if(gamepad1.a && !follower.isBusy())
                {
                    setPathState(30);
                }
                break;
            case 30:
                follower.followPath(line3, true);
                setPathState(32);
                break;
            case 31:
                if(!follower.isBusy())
                {
                    follower.holdPoint(corner4);
                    setPathState(32);
                }
                break;
            case 32:
                if(gamepad1.a && !follower.isBusy())
                {
                    setPathState(40);
                }
                break;
            case 40:
                follower.followPath(line4, true);
                setPathState(42);
                break;
            case 41:
                if(!follower.isBusy())
                {
                    follower.holdPoint(startPose);
                    setPathState(42);
                }
                break;
            case 42:
                if(gamepad1.a && !follower.isBusy())
                {
                    setPathState(10);
                }
                break;
        }
        //if (pathTimer.getElapsedTimeSeconds() > 2.6) {
        //    setPathState(12);
        //}
    }

    /** This switch is called continuously and runs the necessary actions, when finished, it will set the state to -1.
     * (Therefore, it will not run the action continuously) **/
    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                // put code here
                break;
            case 1:
                // put code here
                break;
        }
    }

    /** This switch is called continuously and runs the claw actions, when finished, it will set the state to -1.
     * (Therefore, it will not run the action continuously) **/
    public void clawUpdate() {
        switch (clawState) {
            case 0:
                // put code here
                break;
            case 1:
                // put code here
                break;
        }
    }


    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    public void setActionState(int aState) {
        actionState = aState;
        pathTimer.resetTimer();
        autonomousActionUpdate();
    }

    public void setClawState(int cState) {
        clawState = cState;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the actions and movement of the robot
        follower.update();
        autonomousPathUpdate();
        autonomousActionUpdate();
        clawUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("follower is busy", follower.isBusy());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        // After 4 Seconds, Robot Initialization is complete
        if (opmodeTimer.getElapsedTimeSeconds() > 4) {
            telemetry.addData("Init", "Finished");
        }
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(42);
        setActionState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}