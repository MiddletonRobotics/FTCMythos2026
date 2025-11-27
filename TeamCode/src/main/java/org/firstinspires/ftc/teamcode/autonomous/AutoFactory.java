package org.firstinspires.ftc.teamcode.autonomous;

import java.util.ArrayList;

public class AutoFactory {

    private static final ArrayList<AutoRoutine> routines = new ArrayList<>();

    public static void register(AutoRoutine routine) {
        routines.add(routine);
    }

    public static ArrayList<AutoRoutine> getRoutines() {
        return routines;
    }
}