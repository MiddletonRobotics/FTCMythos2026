package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.CommandOpMode;
import org.firstinspires.ftc.library.command.CommandScheduler;
import org.firstinspires.ftc.library.math.Pair;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@Autonomous(name="SelectableAutonomous", group="Auto", preselectTeleOp="RobotController")
public class SelectableAutonomous extends CommandOpMode {
    private Location selectedLocation = Location.CLOSE;
    private Auto selectedAuto = Auto.IDLE;
    private GlobalConstants.AllianceColor selectedAlliance = GlobalConstants.AllianceColor.BLUE;

    private Drivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private AutoChooser autoChooser;

    private int selectionIndex = 0;
    private static final int totalCategories = 3;

    private boolean lastUp, lastDown, lastLeft, lastRight, lastTriangle;
    private boolean isLockedIn = false;
    private boolean hasBeenScheduled = false;

    @Override
    public void initialize() {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        autoChooser = new AutoChooser(drivetrain, intake, transfer, shooter);

        telemetry.addLine("Use DPAD + Triangle to choose Autonomous.");
        telemetry.update();
    }

    @Override
    public void initialize_loop() {
        boolean triangle = gamepad1.triangle;   

        if (triangle && !lastTriangle && !hasBeenScheduled) {
            isLockedIn = true;
            hasBeenScheduled = true;
            scheduleRoutine();
        } else if(!isLockedIn) {
            readInputs();
            drawUI();
        }

        lastTriangle = triangle;
    }

    /** Handle gamepad navigation */
    private void readInputs() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if(up && !lastUp) {
            selectionIndex = (selectionIndex - 1 + totalCategories) % totalCategories;
        }

        if(down && !lastDown) {
            selectionIndex = (selectionIndex + 1) % totalCategories;
        }

        switch (selectionIndex) {
            case 0: // Location
                if(right && !lastRight)
                    selectedLocation = cycleRight(selectedLocation, Location.values());
                if(left && !lastLeft)
                    selectedLocation = cycleLeft(selectedLocation, Location.values());
                break;

            case 1: // Auto Type
                if(right && !lastRight)
                    selectedAuto = cycleRight(selectedAuto, Auto.values());
                if(left && !lastLeft)
                    selectedAuto = cycleLeft(selectedAuto, Auto.values());
                break;

            case 2: // Alliance
                if(right && !lastRight)
                    selectedAlliance = cycleRight(selectedAlliance, GlobalConstants.AllianceColor.values());
                if(left && !lastLeft)
                    selectedAlliance = cycleLeft(selectedAlliance, GlobalConstants.AllianceColor.values());
                break;
        }

        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;
    }

    /** Draw the selection menu on Driver Hub */
    private void drawUI() {
        telemetry.addLine("=== Autonomous Selection ===");

        telemetry.addLine((selectionIndex == 0 ? "> " : "  ") + "Starting Location: " + selectedLocation);
        telemetry.addLine((selectionIndex == 1 ? "> " : "  ") + "Auto Type:        " + selectedAuto);
        telemetry.addLine((selectionIndex == 2 ? "> " : "  ") + "Alliance Color:   " + selectedAlliance);

        telemetry.addLine("");
        telemetry.addLine("DPAD ←/→ to change value");
        telemetry.addLine("DPAD ↑/↓ to change category");
        telemetry.addLine("A to confirm");

        telemetry.update();
    }

    public void scheduleRoutine() {
        GlobalConstants.opModeType = GlobalConstants.OpModeType.AUTONOMOUS;
        GlobalConstants.allianceColor = selectedAlliance;

        Pair<Pose, Command> routine = autoChooser.getDesiredProgram(selectedLocation, selectedAuto);

        if(routine == null) {
            telemetry.addLine("ERROR: No auto routine found for selection!");
            telemetry.update();
        }

        assert routine != null;
        drivetrain.setStartingPose(routine.getFirst());

        if(!CommandScheduler.getInstance().isScheduled(routine.getSecond())) {
            schedule(routine.getSecond());
        }

        telemetry.addLine("Selections Locked! Starting Auto Initialization...");
        telemetry.addData("Location", selectedLocation);
        telemetry.addData("Auto Type", selectedAuto);
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.update();
    }

    /** Helper to cycle right through an enum */
    private <T> T cycleRight(T current, T[] values) {
        int index = -1;
        for (int i = 0; i < values.length; i++) {
            if (values[i] == current) index = i;
        }

        return values[(index + 1) % values.length];
    }

    /** Helper to cycle left through an enum */
    private <T> T cycleLeft(T current, T[] values) {
        int index = -1;
        for (int i = 0; i < values.length; i++) {
            if (values[i] == current) index = i;
        }

        return values[(index - 1 + values.length) % values.length];
    }
}