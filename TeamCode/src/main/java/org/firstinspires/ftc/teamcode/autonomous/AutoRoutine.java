package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.math.geometry.Pose2d;

public class AutoRoutine {
    private final Auto auto;
    private final String name;

    public AutoRoutine(final Auto auto, final String name, Function<AutoFactory, Pair<Pose2d, Command>> commandFactory) {
        this.auto = auto;
        this.name = name;
    }

    public Auto getAuto() {
        return auto;
    }

    public String getName() {
        return name;
    }

    /**
     * Construct the {@link Command} for this program from the provided {@link AutoFactory}
     *
     * @param autoFactory
     * The {@link AutoFactory} to use when creating the {@link Command}
     * @return
     * The {@link Command} for this program from the provided {@link AutoFactory}
     */
    public Command getCommand(final AutoFactory autoFactory) {
        return commandFactory.apply(autoFactory).getSecond();
    }

    public Pose2d getStartingPose(final AutoFactory autoFactory) {
        return commandFactory.apply(autoFactory).getFirst();
    }
}
