package frc.robot.subsystems;

import org.usfirst.frc.team4999.lights.Display;
import org.usfirst.frc.team4999.lights.NeoPixels;
import org.usfirst.frc.team4999.lights.animations.AnimationSequence;
import org.usfirst.frc.team4999.lights.animations.Snake;
import org.usfirst.frc.team4999.lights.animations.Stack;
import org.usfirst.frc.team4999.lights.compositor.AnimationCompositor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import org.usfirst.frc.team4999.lights.AddressableLEDDisplay;
import org.usfirst.frc.team4999.lights.AsyncAnimator;
import org.usfirst.frc.team4999.lights.Color;
import org.usfirst.frc.team4999.lights.ColorTools;

public class LEDSubsystem {
    private Display display;
    private AsyncAnimator animator;
    private AnimationCompositor compositor;

    private AnimationCompositor.View baseView;

    private static Color[] rainbowcolors = {
        new Color(72, 21, 170),
        new Color(55, 131, 255),
        new Color(77, 233, 76),
        new Color(255, 238, 0),
        new Color(255, 140, 0),
        new Color(246, 0, 0)
    };

    Color[] rainbowTails = ColorTools.getColorTails(rainbowcolors, Color.BLACK, 12, 20);
    Color[] momentumTails = ColorTools.getColorTails(
        new Color[] {Color.MOMENTUM_BLUE, Color.MOMENTUM_PURPLE},
        Color.BLACK, 24, 32
    );

    AnimationSequence mainAnimation = new AnimationSequence(
        new AnimationSequence.AnimationSequenceMember(
            new Snake(20, rainbowTails),
            5000
        ),
        new AnimationSequence.AnimationSequenceMember(
            new Snake(15, ColorTools.getSmearedColors(rainbowcolors, 16)),
            1500
        ),
        new AnimationSequence.AnimationSequenceMember(
            new Snake(20, momentumTails),
            5000
        ),
        new AnimationSequence.AnimationSequenceMember(
            new Stack(20, 60, rainbowcolors),
            1500
        )
    );

    public LEDSubsystem() {
        try {
            this.display = new AddressableLEDDisplay(0, 120);
            this.animator = new AsyncAnimator(display);
            this.compositor = new AnimationCompositor(animator);
        } catch(RuntimeException e) {
            DriverStation.reportError("Error instantiating LEDSubsystem: " + e.getMessage(), e.getStackTrace());
            return;
        }

        baseView = compositor.getOpaqueView(mainAnimation, 0);
    }

    public AnimationCompositor getCompositor() {
        return compositor;
    }
}
