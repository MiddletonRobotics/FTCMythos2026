package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.lights.LightsManager;
import com.bylazar.lights.PanelsLights;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.Optional;

@TeleOp(name="TestingOpMode", group="TeleOp")
public class TestingOpMode extends OpMode {
    public Intake intake;
    public Turret turret;
    public LED led;
    public Shooter shooter;
    public Drivetrain drivetrain;

    @IgnoreConfigurable
    private TelemetryManager telemetryManager;

    @IgnoreConfigurable
    static LightsManager lightsManager;

    @Override
    public void init() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        lightsManager = PanelsLights.INSTANCE.getLights();

        drivetrain = new Drivetrain(hardwareMap, telemetryManager);
        intake = new Intake(hardwareMap, telemetryManager);
        shooter = new Shooter(hardwareMap, telemetryManager);
        turret = new Turret(hardwareMap, telemetryManager);
        led = new LED(hardwareMap, telemetryManager, lightsManager);

        GlobalConstants.allianceColor= GlobalConstants.AllianceColor.RED;
        GlobalConstants.opModeType = GlobalConstants.OpModeType.TELEOP;

        drivetrain.setStartingPose(DrivetrainConstants.decideToFlipPose(GlobalConstants.allianceColor, DrivetrainConstants.kCloseGoalStartingPoseBlue));
        drivetrain.update();
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger > 0.5) {
            shooter.setVelocitySetpoint(ShooterConstants.kTuningFlywheelVelocitySetpoint);
        } else if(gamepad1.right_trigger < 0.5) {
            shooter.setVelocitySetpoint(1);
        }

        if(gamepad1.left_trigger > 0.5) {
            intake.setOpenLoopSetpoint(1);
        } else if(gamepad1.left_bumper) {
            intake.setOpenLoopSetpoint(-1);
        } else if(gamepad1.left_trigger < 0.5) {
            intake.setOpenLoopSetpoint(0);
        }

        if(gamepad1.square) {
            shooter.setHoodPosition(ShooterConstants.kTuningHoodPositionSetpoint);
        } else if(!gamepad1.square) {
            shooter.setHoodPosition(0);
        }

        if(gamepad1.dpad_down) {
            turret.setPosition(1.57);
            led.setSolid(LEDConstants.ColorValue.BLUE);
        } else if(!gamepad1.dpad_down) {
            turret.setPosition(0);
            led.setSolid(LEDConstants.ColorValue.GREEN);
        }

        drivetrain.periodic();
        shooter.periodic();
        turret.periodic();
        drivetrain.update();

        telemetryManager.addData("Distance Reading", drivetrain.getDistanceToPose3D(turret.getTargetPose(GlobalConstants.getCurrentAllianceColor()), 38, 12));
        telemetryManager.update(telemetry);
    }
}
