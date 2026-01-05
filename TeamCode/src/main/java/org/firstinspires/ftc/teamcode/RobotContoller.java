package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.lights.LightsManager;
import com.bylazar.lights.PanelsLights;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.CommandOpMode;
import org.firstinspires.ftc.library.command.CommandScheduler;
import org.firstinspires.ftc.library.command.RunCommand;
import org.firstinspires.ftc.library.command.button.Trigger;
import org.firstinspires.ftc.library.gamepad.GamepadEx;
import org.firstinspires.ftc.library.gamepad.GamepadKeys;
import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.Auto;
import org.firstinspires.ftc.teamcode.autonomous.Location;
import org.firstinspires.ftc.teamcode.command_factories.IntakeFactory;
import org.firstinspires.ftc.teamcode.command_factories.ShooterFactory;
import org.firstinspires.ftc.teamcode.command_factories.TransferFactory;
import org.firstinspires.ftc.teamcode.command_factories.TurretFactory;
import org.firstinspires.ftc.teamcode.commands.ConstrainedFlashCommand;
import org.firstinspires.ftc.teamcode.commands.TeleopMecanum;
import org.firstinspires.ftc.teamcode.commands.TurretPositionSetpoint;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.LEDConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utilities.SavedConfiguration;

@TeleOp(name="RobotController", group="TeleOp")
public class RobotContoller extends CommandOpMode {
    private Drivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private Vision vision;
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
        led = new LED(hardwareMap, telemetryManager, lightsManager);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        savedLocation = SavedConfiguration.selectedLocation;
        savedAutonomousRoutine = SavedConfiguration.selectedAuto;
        savedAllianceColor = SavedConfiguration.selectedAlliance;

        teleopInitTime = getRuntime();
        configLocked = false;
        teleopInitialized = false;

        shooter.onInitialization();
        transfer.onInitialization(true, true);

        new Trigger(() -> driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(IntakeFactory.openLoopSetpointCommand(intake, () -> 1))
                .whenInactive(IntakeFactory.openLoopSetpointCommand(intake, () -> 0));

        new Trigger(() -> driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.75))
                .whenInactive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

        new Trigger(() -> transfer.secondCSDistance() < 2.5).whenActive(() -> led.setSolid(LEDConstants.ColorValue.GREEN));
        new Trigger(() -> transfer.firstCSDistance() < 2.5).whenActive(() -> led.setSolid(LEDConstants.ColorValue.YELLOW));

        driverController.getGamepadButton(GamepadKeys.Button.SQUARE).toggleWhenActive(
                () -> shooter.setHoodPosition(0.65),
                () -> shooter.setHoodPosition(ShooterConstants.hoodIdlePosition)
        );

        driverController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(IntakeFactory.openLoopSetpointCommand(intake, () -> -1))
                .whenReleased(IntakeFactory.openLoopSetpointCommand(intake, () -> 0));

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenActive(() -> shooter.setHoodPosition(shooter.getHoodTargetPosition() + 0.001));

        driverController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(ShooterFactory.openLoopSetpointCommand(shooter, () -> -0.3))
                .whenReleased(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

        driverController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(new ConstrainedFlashCommand(led, LEDConstants.ColorValue.ORANGE, () -> 125, () -> 20));

        new Trigger(() -> operatorController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 1));

        new Trigger(() -> operatorController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5)
                .whenActive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

        operatorController.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(TransferFactory.runKickerCycle(transfer));

        operatorController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).toggleWhenPressed(
                () -> transfer.setBlockerPosition(TransferConstants.blockerAllowPosition),
                () -> transfer.setBlockerPosition(TransferConstants.blockerIdlePosition)
        );

        operatorController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(() -> turret.setManualPower(0.4))
            .whenReleased(() -> turret.setManualPower(0.0));

        operatorController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> turret.setManualPower(-0.4))
                .whenReleased(() -> turret.setManualPower(0.0));

        drivetrain.setDefaultCommand(new TeleopMecanum(
                drivetrain,
                () -> driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> true
        ));
        new  Trigger(() -> operatorController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0.75))
                .whenInactive(ShooterFactory.openLoopSetpointCommand(shooter, () -> 0));

        schedule(
                new RunCommand(() -> telemetryManager.update(telemetry)),
                new RunCommand(led::update)
        );
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
        }

        telemetryManager.update(telemetry);
        led.update();
    }

    public void initializeOpMode() {
        if (teleopInitialized) return;
        teleopInitialized = true;
    }

    @SuppressLint("DefaultLocale")
    public void drawUnlockedUI(double elaspedTimeSinceStarted) {
        telemetryManager.addData("Saved Location", savedLocation);
        telemetryManager.addData("Saved Autonomous Routine", savedAutonomousRoutine);
        telemetryManager.addData("Saved Alliance Selection", savedAllianceColor);

        telemetryManager.addLine("");
        telemetryManager.addLine("DPAD ←/→ change value");
        telemetryManager.addLine("DPAD ↑/↓ change category");
        telemetryManager.addLine("");

        telemetry.addData("Time Left", String.format("%.1f", CONFIG_TIMEOUT_SEC - elaspedTimeSinceStarted));
        telemetryManager.addLine("Locks automatically at 4s");
    }

    public void drawLockedUI() {
        telemetry.addLine("=== Configuration Locked ===");
        telemetryManager.addData("Location", savedLocation);
        telemetryManager.addData("Auto", savedAutonomousRoutine);
        telemetryManager.addData("Alliance", savedAllianceColor);
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
}
