package org.firstinspires.ftc.teamcode.decode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "shootertest", group = "decode")
public class ShooterTest extends OpMode {
    DcMotor intake;
    DcMotorEx shooter;
    DcMotorEx shooterRight;

    Servo lifter;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER ,new PIDFCoefficients(300, 0, 0, 9.5));
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterR");
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        lifter = hardwareMap.get(Servo.class, "lifter");

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public double intakePower = 0;
    public double shooterSpeedSetpoint = 0;
    public double shooterSpeed;

    double LIFTER_UP = 0.384;
    double LIFTER_DOWN = 0.459;

    double lifterPos = LIFTER_DOWN;

    Telemetry telemetryA;

    @Override
    public void loop() {
        intakePower += gamepad1.left_stick_y * -0.001;
        intakePower = MathFunctions.clamp(intakePower,-1, 1);

        shooterSpeed = shooter.getVelocity();

        shooterSpeedSetpoint += gamepad1.right_stick_y * 1;
        //shooterSpeedSetpoint = MathFunctions.clamp(shooterSpeedSetpoint,-30000, 30000);

        if(gamepad1.a)
            lifterPos = LIFTER_UP;

        if(gamepad1.b)
            lifterPos = LIFTER_DOWN;

        lifterPos += (gamepad1.right_trigger - gamepad1.left_trigger) * 0.001;
        lifterPos = MathFunctions.clamp(lifterPos, 0, 1);

        telemetryA.addData("intake power", intakePower);
        telemetryA.addData("shooter left power", shooter.getPower());
        telemetryA.addData("shooter right power", shooterRight.getPower());
        telemetryA.addData("shooter speed setpoint", shooterSpeedSetpoint);
        telemetryA.addData("shooter speed", shooterSpeed);

        telemetryA.addData("lifter pos", lifterPos);
        telemetryA.update();

        intake.setPower(intakePower);

        shooter.setVelocity(shooterSpeedSetpoint);
        shooterRight.setPower(shooter.getPower());

        lifter.setPosition(lifterPos);
    }
}
