package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.teleop.Drive;
import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Outtake;

@Disabled
@Autonomous(name = "1 specimen auto", group = "Autonomous", preselectTeleOp = "Master Tele-op")
public class OneSpecAuto extends OpMode {
    private static final double MOVEMENT_POWER = 0.4;

    private Outtake outtake;
    private Intake intake;
    private Drive drive;
    private Timer timer;
    private int state;
    private int startDelay = -1;
    private boolean onsSetState = false;
    private boolean onsBumper = false;


    @Override
    public void init()
    {
        timer = new Timer();
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        drive = new Drive(hardwareMap, gamepad1);
        setState(-1);
    }

    @Override
    public void init_loop()
    {
        if(gamepad1.dpad_right && !onsBumper) {
            startDelay++;
            onsBumper = true;
        }
        else if(gamepad1.dpad_left && !onsBumper) {
            startDelay--;
            onsBumper = true;
        } else if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
            onsBumper = false;
        }

        startDelay = (int)MathFunctions.clamp(startDelay, 0, 15);

        telemetry.addData("start delay", startDelay);
        telemetry.update();
    }

    @Override
    public void start()
    {
        timer.resetTimer();
    }

    @Override
    public void loop()
    {
        autoLoop();

        outtake.update();
        intake.horizontalSlidesUpdate();

        telemetry.addData("auto state", state);
        telemetry.addData("outtake state", outtake.getState());
        telemetry.addData("intake state", intake.getState());
        telemetry.update();
    }

    private void autoLoop()
    {

        switch (state)
        {
            case -1:
                if(timer.getElapsedTimeSeconds() > startDelay){
                    setState(0);
                }
                break;

            case 0:
                drive.autoMovement(0, MOVEMENT_POWER - 0.1, 0);
                if(onsSetState){
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN_READY_HIGH);
                    onsSetState = false;
                }

                if(timer.getElapsedTimeSeconds() > 3.5){
                    outtake.setState(OuttakeConstants.SCORE_SPECIMEN);
                    setState(1);
                }
                break;

            case 1:

                if(timer.getElapsedTimeSeconds() > 0.5){
                    drive.autoMovement(0, -MOVEMENT_POWER, 0);
                }


                if(timer.getElapsedTimeSeconds() > 1.25){
                    outtake.setState(OuttakeConstants.TRANSFER_INTAKE_READY);
                    setState(2);
                }
                break;

            case 2:
                drive.autoMovement(-MOVEMENT_POWER, 0, 0);
                if(timer.getElapsedTimeSeconds() > 5){
                    setState(10);
                }
                break;

            case 10:
                drive.autoMovement(0, 0, 0);
                telemetry.addLine("done!");
                break;
        }
    }

    private void setState(int state){
        this.state = state;
        onsSetState = true;
        timer.resetTimer();
    }
}
