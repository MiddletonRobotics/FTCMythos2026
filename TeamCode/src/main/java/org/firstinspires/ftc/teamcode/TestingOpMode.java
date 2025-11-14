package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    public MotorEx intakeMotor;
    public MotorEx turretMotor;
    public MotorEx shooterMotor;
    public MotorEx ascentMotor;
    public ServoEx hoodServo;
    public Follower follower;
    public ServoEx kickerServo;
    public ServoEx blockerServo;

    public boolean toogle = false;

    @Override
    public void init() {
        intakeMotor = new MotorEx(hardwareMap, IntakeConstants.intakeMotorID, Motor.GoBILDA.RPM_435);
        turretMotor = new MotorEx(hardwareMap, TurretConstants.turretMotorID, Motor.GoBILDA.BARE);
        shooterMotor = new MotorEx(hardwareMap, ShooterConstants.shooterMotorID, Motor.GoBILDA.BARE);
        ascentMotor = new MotorEx(hardwareMap, "ascentMotor", Motor.GoBILDA.RPM_312);
        hoodServo = new ServoEx(hardwareMap, "hoodServo");
        kickerServo = new ServoEx(hardwareMap, "kickerServo");
        blockerServo = new ServoEx(hardwareMap, "blockServo");

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
            turretMotor.set(0.2);
        } else if(!gamepad1.left_bumper) {
            turretMotor.set(0.0);
        }

        if(gamepad1.right_bumper) {
            turretMotor.set(-0.2);
        } else if(!gamepad1.right_bumper) {
            turretMotor.set(0.0);
        }

        if(gamepad1.right_trigger > 0.5) {
            shooterMotor.set(-1.0);
        } else if(gamepad1.right_trigger < 0.5) {
            shooterMotor.set(0.0);
        }

            if(gamepad1.left_trigger > 0.5) {
            intakeMotor.set(-1.0);
        } else if(gamepad1.left_trigger < 0.5) {
            intakeMotor.set(0.0);
        }

        if(gamepad1.xWasPressed()) {
            hoodServo.set(2.0);
        } else if(!gamepad1.x) {
            hoodServo.set(-1);
        }

        if(gamepad1.yWasPressed()) {
            kickerServo.set(0.5);
        } else if(!gamepad1.y) {
            kickerServo.set(0);
        }

        if(gamepad1.xWasPressed()) {
            blockerServo.set(0);
        } else if(!gamepad1.x) {
            blockerServo.set(0.3);
        }


        follower.update();
        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);

        telemetry.addData("Follower Pose X", follower.getPose().getX());
        telemetry.addData("Follower Pose Y", follower.getPose().getY());
        telemetry.addData("Follower Pose Rotation", follower.getPose().getHeading());
        telemetry.update();
    }
}
