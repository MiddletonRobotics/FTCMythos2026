package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@TeleOp(name="TestingOpMode", group="TeleOp")
public class TestingOpMode extends OpMode {
    public DcMotorEx intakeMotor;
    public DcMotorEx turretMotor;
    public DcMotorEx shooterMotor;
    public DcMotorEx ascentMotor;

    public Servo hoodServo;
    public Follower follower;
    public Servo kickerServo;
    public Servo blockerServo;

    public boolean toogle = false;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConstants.intakeMotorID);
        turretMotor = hardwareMap.get(DcMotorEx.class, TurretConstants.turretMotorID);
        shooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.shooterMotorID);
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
        if(gamepad1.left_bumper) {
            turretMotor.setPower(0.2);
        } else if(!gamepad1.left_bumper) {
            turretMotor.setPower(0.0);
        }

        if(gamepad1.right_bumper) {
            turretMotor.setPower(-0.2);
        } else if(!gamepad1.right_bumper) {
            turretMotor.setPower(0.0);
        }

        if(gamepad1.right_trigger > 0.5) {
            shooterMotor.setPower(-1.0);
        } else if(gamepad1.right_trigger < 0.5) {
            shooterMotor.setPower(0.0);
        }

            if(gamepad1.left_trigger > 0.5) {
            intakeMotor.setPower(-1.0);
        } else if(gamepad1.left_trigger < 0.5) {
            intakeMotor.setPower(0.0);
        }

        if(gamepad1.xWasPressed()) {
            hoodServo.setPosition(2.0);
        } else if(!gamepad1.x) {
            hoodServo.setPosition(-1);
        }

        if(gamepad1.yWasPressed()) {
            kickerServo.setPosition(0.5);
        } else if(!gamepad1.y) {
            kickerServo.setPosition(0);
        }

        if(gamepad1.xWasPressed()) {
            blockerServo.setPosition(0);
        } else if(!gamepad1.x) {
            blockerServo.setPosition(0.3);
        }


        follower.update();
        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);

        telemetry.addData("Follower Pose X", follower.getPose().getX());
        telemetry.addData("Follower Pose Y", follower.getPose().getY());
        telemetry.addData("Follower Pose Rotation", follower.getPose().getHeading());
        telemetry.update();
    }
}
