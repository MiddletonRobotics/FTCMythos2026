package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Deprecated
@Disabled
@TeleOp(name="TestingOpMode", group="TeleOp")
public class TestingOpMode extends OpMode {
    public DcMotorEx intakeMotor;
    public Turret turret;
    public LED led;
    public Shooter shooter;
    public DcMotorEx ascentMotor;

    public Servo hoodServo;
    public Follower follower;
    public Servo kickerServo;
    public Servo blockerServo;

    public boolean toogle = false;

    private Telemetry telemetryA;

    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConstants.intakeMotorID);
        shooter = new Shooter(hardwareMap, telemetryA);
        turret = new Turret(hardwareMap, telemetryA);
        led = new LED(hardwareMap, telemetryA);
        ascentMotor = hardwareMap.get(DcMotorEx.class, "ascentMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
        blockerServo = hardwareMap.get(Servo.class, "blockServo");

        kickerServo.setDirection(Servo.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0));
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger > 0.5) {
            shooter.setVelocitySetpoint(6000);
        } else if(gamepad1.right_trigger < 0.5) {
            shooter.setVelocitySetpoint(0);
        }

        if(gamepad1.left_trigger > 0.5) {
            intakeMotor.setPower(-1.0);
        } else if(gamepad1.left_trigger < 0.5) {
            intakeMotor.setPower(0.0);
        }

        if(gamepad1.dpad_down) {
            hoodServo.setPosition(1.0);
        } else if(!gamepad1.dpad_down) {
            hoodServo.setPosition(0.4);
        }

        if(gamepad1.yWasPressed()) {
            kickerServo.setPosition(0.5);
        } else if(!gamepad1.y) {
            kickerServo.setPosition(0);
        }

        if(gamepad1.dpad_up) {
            blockerServo.setPosition(TransferConstants.blockerAllowPosition);
        } else if(!gamepad1.dpad_up) {
            blockerServo.setPosition(TransferConstants.blockerIdlePosition);
        }

        if(gamepad1.dpad_right) {
            turret.setPosition(0);
            led.enableSolidColor(LEDConstants.ColorValue.BLUE);
        } else if(!gamepad1.dpad_right) {
            turret.setPosition(1700);
            led.enableSolidColor(LEDConstants.ColorValue.GREEN);
        }

        if(gamepad1.dpad_left) {
            blockerServo.setPosition(TransferConstants.blockerAllowPosition);
        } else if(!gamepad1.dpad_left) {
            blockerServo.setPosition(TransferConstants.blockerIdlePosition);
        }

        shooter.periodic();


        follower.update();
        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);

        telemetryA.addData("Follower Pose X", follower.getPose().getX());
        telemetryA.addData("Follower Pose Y", follower.getPose().getY());
        telemetryA.addData("Follower Pose Rotation", follower.getPose().getHeading());
        telemetryA.update();
    }
}
