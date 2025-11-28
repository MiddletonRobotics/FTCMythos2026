package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.math.Pair;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class AutoChooser {
    private final Map<Auto, AutoRoutine> routines;
    private final Map<GlobalConstants.AllianceColor, Map<Auto, Pair<Pose, Command>>> commandCache;
    private final Map<GlobalConstants.AllianceColor, AutoFactory> autoFactories;

    private final List<Map.Entry<String, Auto>> options = new ArrayList<>();
    private int currentIndex = 0;

    /**
     * Create a new <code>AutoChooser</code>
     *
     * @return A new <code>AutoChooser</code> populated with the programs defined in the private field {@link AutoChooser#routines}.
     */
    public static AutoChooser create(final Drivetrain drivetrain, final Intake intake, final Transfer transfer, final Shooter shooter) {
        List<AutoRoutine> autoRoutineList = List.of(
                new AutoRoutine(Auto.IDLE, "Idle", AutoFactory::initializeIdle)
                // TODO: Add more AutoRoutine entries here
        );

        Map<Auto, AutoRoutine> routines = new HashMap<>();
        for (AutoRoutine routine : autoRoutineList) routines.put(routine.getAuto(), routine);

        Map<GlobalConstants.AllianceColor, AutoFactory> factories = new HashMap<>();
        for (GlobalConstants.AllianceColor a : GlobalConstants.AllianceColor.values()) factories.put(a, new AutoFactory(a, drivetrain, intake, transfer, shooter));

        AutoChooser autoChooser = new AutoChooser(routines, factories);

        autoRoutineList.forEach(routine -> {
            if(routine.getAuto() == Auto.IDLE) {
                autoChooser.setDefaultOption(routine.getName(), routine.getAuto());
            } else {
                autoChooser.addOption(routine.getName(), routine.getAuto());
            }
        });

        return autoChooser;
    }

    private AutoChooser(final Map<Auto, AutoRoutine> routines, final Map<GlobalConstants.AllianceColor, AutoFactory> autoFactories) {
        this.routines = routines;
        this.autoFactories = autoFactories;
        commandCache = Stream.of(GlobalConstants.AllianceColor.values())
            .map(alliance -> Map.entry(alliance, new HashMap<Auto, Pair<Pose, Command>>()))
            .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));
    }

    public void setDefaultOption(String name, Auto auto) {
        options.clear();
        options.add(Map.entry(name, auto));
        currentIndex = 0;
    }

    public void addOption(String name, Auto auto) {
        options.add(Map.entry(name, auto));
    }

    public Auto getSelected() {
        return options.get(currentIndex).getValue();
    }

    public String getSelectedName() {
        return options.get(currentIndex).getKey();
    }

    public void next() {
        currentIndex = (currentIndex + 1) % options.size();
    }

    public void previous() {
        currentIndex = (currentIndex - 1 + options.size()) % options.size();
    }

    public Command getSelectedCommand(GlobalConstants.AllianceColor alliance) {
        Auto selected = getSelected();
        return commandCache.get(alliance).get(selected).getSecond();
    }

    public Pose getStartingPose(GlobalConstants.AllianceColor alliance) {
        Auto selected = getSelected();
        return commandCache.get(alliance).get(selected).getFirst();
    }

    public AutoRoutine getRoutine() {
        return routines.get(getSelected());
    }
}
