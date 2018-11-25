package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class GOFVuforiaLocalizer extends VuforiaLocalizerImpl {

    private boolean closed;

    public GOFVuforiaLocalizer(Parameters parameters) {
        super(parameters); // When this object is instantiated, instantiate a regular VuforiaLocalizer object with the same parameters
        closed = false;
    }

    @Override
    public void close() { // Add a public close method to vuforia
        if(!closed) {
            super.close();
        }
        closed = true;
    }
}