package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.lights.LightsManager;
import com.bylazar.lights.PanelsLights;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.library.command.CommandOpMode;
import org.firstinspires.ftc.library.command.CommandScheduler;
import org.firstinspires.ftc.library.command.RunCommand;
import org.firstinspires.ftc.library.gamepad.GamepadEx;
import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.library.math.geometry.Rotation2d;
import org.firstinspires.ftc.library.vision.FiducialData3D;
import org.firstinspires.ftc.teamcode.autonomous.Auto;
import org.firstinspires.ftc.teamcode.autonomous.Location;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.SavedConfiguration;

import java.util.Optional;

@TeleOp(name="RobotController", group="TeleOp")
public class RobotContoller extends CommandOpMode {
    private Drivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private Vision vision;
    private Lift lift;
    private LED led;

    private GamepadEx driverController;
    private GamepadEx operatorController;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    @IgnoreConfigurable
    static LightsManager lightsManager;

    private Location savedLocation;
    private Auto savedAutonomousRoutine;
    private GlobalConstants.AllianceColor savedAllianceColor;
    private Pose savedAutonomousEndingPosition;

    private int selectionIndex = 0;
    private static final int TOTAL_CATEGORIES = 3;

    private static final double CONFIG_TIMEOUT_SEC = 4.0;
    private double teleopInitTime;

    private boolean configLocked = false;
    private boolean teleopInitialized = false;
    private boolean lastTriangle;
    private boolean lastUp, lastDown, lastLeft, lastRight;

    @Override
    public void initialize() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        lightsManager = PanelsLights.INSTANCE.getLights();

        drivetrain = new Drivetrain(hardwareMap, telemetryManager);
        intake = new Intake(hardwareMap, telemetryManager);
        transfer = new Transfer(hardwareMap, telemetryManager);
        shooter = new Shooter(hardwareMap, telemetryManager);
        turret = new Turret(hardwareMap, telemetryManager);
        vision = new Vision(hardwareMap, telemetryManager);
        lift = new Lift(hardwareMap, telemetryManager);
        led = new LED(hardwareMap, telemetryManager, lightsManager);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        GlobalConstants.opModeType = GlobalConstants.OpModeType.TELEOP;
        savedLocation = SavedConfiguration.selectedLocation;
        savedAutonomousRoutine = SavedConfiguration.selectedAuto;
        savedAllianceColor = SavedConfiguration.selectedAlliance;
        savedAutonomousEndingPosition = SavedConfiguration.finalDrivetrainPose;

        teleopInitTime = getRuntime();
        configLocked = false;
        teleopInitialized = false;

        drivetrain.setStartingPose(savedAutonomousEndingPosition);
        shooter.onInitialization();
        transfer.onInitialization(true, true);

        TeleopBindings.configureBindings(driverController, operatorController, drivetrain, intake, transfer, shooter, turret, lift, led);
        TeleopBindings.configureDefaultCommands(driverController, operatorController, drivetrain, intake, transfer, shooter, turret, led);

        schedule(new RunCommand(() -> telemetryManager.update(telemetry)));
    }

    private void readInputs() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if (up && !lastUp) selectionIndex = (selectionIndex - 1 + TOTAL_CATEGORIES) % TOTAL_CATEGORIES;
        if (down && !lastDown) selectionIndex = (selectionIndex + 1) % TOTAL_CATEGORIES;

        switch (selectionIndex) {
            case 0:
                if (right && !lastRight) savedLocation = cycleRight(savedLocation, Location.values());
                if (left && !lastLeft) savedLocation = cycleLeft(savedLocation, Location.values());
                break;
            case 1:
                if (right && !lastRight) savedAutonomousRoutine = cycleRight(savedAutonomousRoutine, Auto.values());
                if (left && !lastLeft) savedAutonomousRoutine = cycleLeft(savedAutonomousRoutine, Auto.values());
                break;
            case 2:
                if (right && !lastRight) savedAllianceColor = cycleRight(savedAllianceColor, GlobalConstants.AllianceColor.values());
                if (left && !lastLeft) savedAllianceColor = cycleLeft(savedAllianceColor, GlobalConstants.AllianceColor.values());
                break;
        }

        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;
    }

    @Override
    public void initialize_loop() {
        readInputs();

        double elapsed = getRuntime() - teleopInitTime;

        if (!configLocked && elapsed >= CONFIG_TIMEOUT_SEC) {
            configLocked = true;
            initializeOpMode();
        }

        if (!configLocked) {
            readInputs();
            drawUnlockedUI(elapsed);
        } else {
            drawLockedUI();
            setGlobalSettings();
        }

        telemetryManager.update(telemetry);
        lift.onInitialization();
        led.update();
    }

    private void initializeOpMode() {
        if (teleopInitialized) return;
        teleopInitialized = true;
    }

    @SuppressLint("DefaultLocale")
    private void drawUnlockedUI(double elaspedTimeSinceStarted) {
        telemetryManager.addData("Saved Location", savedLocation);
        telemetryManager.addData("Saved Autonomous Routine", savedAutonomousRoutine);
        telemetryManager.addData("Saved Alliance Selection", savedAllianceColor);

        telemetryManager.addLine("");
        telemetryManager.addLine("DPAD ←/→ change value");
        telemetryManager.addLine("DPAD ↑/↓ change category");
        telemetryManager.addLine("");

        telemetry.addData("Time Left", Optional.of(CONFIG_TIMEOUT_SEC - elaspedTimeSinceStarted));
        telemetryManager.addLine("Locks automatically at 4s");
    }

    private void drawLockedUI() {
        telemetry.addLine("=== Configuration Locked ===");
        telemetryManager.addData("Location", savedLocation);
        telemetryManager.addData("Auto", savedAutonomousRoutine);
        telemetryManager.addData("Alliance", savedAllianceColor);
    }

    private void setGlobalSettings() {
        GlobalConstants.allianceColor = savedAllianceColor;
    }

    private <T> T cycleRight(T current, T[] values) {
        for (int i = 0; i < values.length; i++) {
            if (values[i] == current) return values[(i + 1) % values.length];
        }

        return current;
    }

    private <T> T cycleLeft(T current, T[] values) {
        for (int i = 0; i < values.length; i++) {
            if (values[i] == current) return values[(i - 1 + values.length) % values.length];
        }

        return current;
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        Optional<FiducialData3D> visionData = vision.getEstimatedRobotPose(savedAllianceColor);

        if (visionData.isPresent() && vision.shouldTrustVision(visionData.get(), drivetrain.getPose().getAsPedroPose())) {
            Pose visionPose = visionData.get().getRobotPose();

            Pose2d visionPose2d = new Pose2d(
                    visionPose.getX(),
                    visionPose.getY(),
                    new Rotation2d(visionPose.getHeading())
            );

            telemetryManager.addData(VisionConstants.kSubsystemName + " Estimated Robot Pose X", visionPose2d.getX());
            telemetryManager.addData(VisionConstants.kSubsystemName + " Estimated Robot Pose Y", visionPose2d.getY());
            telemetryManager.addData(VisionConstants.kSubsystemName + " Estimated Robot Pose θ", visionPose2d.getRotation().getRadians());

            //drivetrain.updateWithVision(visionPose2d);
        }
    }
}
