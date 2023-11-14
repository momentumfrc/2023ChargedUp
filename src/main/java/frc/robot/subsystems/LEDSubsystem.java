package frc.robot.subsystems;

import org.usfirst.frc.team4999.lights.Display;
import org.usfirst.frc.team4999.lights.NeoPixels;
import org.usfirst.frc.team4999.lights.animations.Animation;
import org.usfirst.frc.team4999.lights.animations.AnimationSequence;
import org.usfirst.frc.team4999.lights.animations.ClippedAnimation;
import org.usfirst.frc.team4999.lights.animations.Overlay;
import org.usfirst.frc.team4999.lights.animations.Snake;
import org.usfirst.frc.team4999.lights.animations.Solid;
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

    private static final int STOP_1 = 52;
    private static final int STOP_2 = 122;
    private static final int STOP_3 = 193;
    private static final int LED_LENGTH = 240;

    private static Color[] offsetColorArray(Color[] buff, int offset) {
        Color[] ret = new Color[buff.length];
        while(offset < 0) {
            offset += buff.length;
        }
        for(int i = 0; i < buff.length; ++i) {
            ret[(i + offset) % ret.length] = buff[i];
        }
        return ret;
    }

    private static Color[] reverseColorArray(Color[] buff) {
        Color[] ret = new Color[buff.length];
        for(int i = 0; i < buff.length; ++i) {
            ret[ret.length - 1 - i] = buff[i];
        }

        return ret;
    }

    private static Color[] rainbowcolors = {
        new Color(72, 21, 170),
        new Color(55, 131, 255),
        new Color(77, 233, 76),
        new Color(255, 238, 0),
        new Color(255, 140, 0),
        new Color(246, 0, 0)
    };

    Color[] rainbowTails = ColorTools.getColorTails(rainbowcolors, Color.BLACK, 10, 6);
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

    AnimationSequence mainAnimation2 = new AnimationSequence(
        new AnimationSequence.AnimationSequenceMember(new Overlay(
            new ClippedAnimation(new Snake(rainbowTails, 20, true), 0, STOP_1),
            new ClippedAnimation(new Snake(rainbowTails, 20, false), STOP_1, STOP_2),
            new ClippedAnimation(new Snake(rainbowTails, 20, true), STOP_2, STOP_3),
            new ClippedAnimation(new Snake(rainbowTails, 20, false), STOP_3, LED_LENGTH)
        ), 5000),
        new AnimationSequence.AnimationSequenceMember(new Overlay(
            new ClippedAnimation(new Snake(ColorTools.getSmearedColors(rainbowcolors, 16), 15, false), 0, STOP_2),
            new ClippedAnimation(new Snake(ColorTools.getSmearedColors(rainbowcolors, 16), 15, true), STOP_2, LED_LENGTH)
        ), 5000),
        new AnimationSequence.AnimationSequenceMember(new Overlay(
            new ClippedAnimation(new Snake(reverseColorArray(momentumTails), 20, true), 0, STOP_2),
            new ClippedAnimation(new Snake(offsetColorArray(momentumTails, 17), 20, false), STOP_2, LED_LENGTH)
        ), 5000)
    );

    public LEDSubsystem() {
        try {
            this.display = new AddressableLEDDisplay(9, LED_LENGTH);
            this.animator = new AsyncAnimator(display);
            this.compositor = new AnimationCompositor(animator);
        } catch(RuntimeException e) {
            DriverStation.reportError("Error instantiating LEDSubsystem: " + e.getMessage(), e.getStackTrace());
            return;
        }

        baseView = compositor.getOpaqueView(mainAnimation2);
    }

    public AnimationCompositor getCompositor() {
        return compositor;
    }
}
