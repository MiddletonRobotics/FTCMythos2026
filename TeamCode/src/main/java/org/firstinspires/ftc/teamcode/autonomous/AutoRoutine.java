package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.math.Pair;

import java.util.function.Function;


public class AutoRoutine {

    private final Auto auto;
    private final String name;
    private final Function<AutoFactory, Pair<Pose, Command>> factory;

    public AutoRoutine(Auto auto, String name, Function<AutoFactory, Pair<Pose, Command>> factory) {
        this.auto = auto;
        this.name = name;
        this.factory = factory;
    }

    public Auto getAuto() {
        return auto;
    }

    public String getName() {
        return name;
    }

    public Command getCommand(final AutoFactory autoFactory) {
        return factory.apply(autoFactory).getSecond();
    }

    public Pose getStartingPose(final AutoFactory autoFactory) {
        return factory.apply(autoFactory).getFirst();
    }
}
