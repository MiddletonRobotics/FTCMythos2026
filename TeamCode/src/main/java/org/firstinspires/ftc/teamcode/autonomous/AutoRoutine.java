package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.math.Pair;
import org.firstinspires.ftc.library.math.geometry.Pose2d;

import java.util.function.Function;

public class AutoRoutine {
    private final Auto auto;
    private final String name;
    private final Function<AutoFactory, Pair<Pose, Command>> commandFactory;

    public AutoRoutine(final Auto auto, final String name, final Function<AutoFactory, Pair<Pose, Command>> commandFactory) {
        this.auto = auto;
        this.name = name;
        this.commandFactory = commandFactory;
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

    public Pose getStartingPose(final AutoFactory autoFactory) {
        return commandFactory.apply(autoFactory).getFirst();
    }
}
