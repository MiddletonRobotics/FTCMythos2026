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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp(name="TestingOpMode", group="TeleOp")
public class TestingOpMode extends OpMode {
    public Intake intake;
    public Turret turret;
    public LED led;
    public Shooter shooter;
    private Vision vison;

    public Follower follower;


    public boolean toogle = false;

    private Telemetry telemetryA;

    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = new Intake(hardwareMap, telemetryA);
        shooter = new Shooter(hardwareMap, telemetryA);
        turret = new Turret(hardwareMap, telemetryA);
        led = new LED(hardwareMap, telemetryA);

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

        if(gamepad1.dpad_left) {
            turret.setPosition(-Math.PI / 2);
            led.enableSolidColor(LEDConstants.ColorValue.BLUE);
        } else if(!gamepad1.dpad_left) {
            turret.setPosition(0);
            led.enableSolidColor(LEDConstants.ColorValue.GREEN);
        }

        shooter.periodic();
        turret.periodic();

        follower.update();
        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);

        telemetryA.addData("Follower Pose X", follower.getPose().getX());
        telemetryA.addData("Follower Pose Y", follower.getPose().getY());
        telemetryA.addData("Follower Pose Rotation", follower.getPose().getHeading());
        telemetryA.update();
    }
}
