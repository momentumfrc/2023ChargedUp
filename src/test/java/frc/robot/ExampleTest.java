package frc.robot;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Assertions;

import edu.wpi.first.hal.HAL;

public class ExampleTest {
    @BeforeEach
    void setup() {
        assert HAL.initialize(5000, 0);
    }

    @Test
    void testContainer() {
        RobotContainer container = new RobotContainer();
        Assertions.assertNotNull(container);
    }
}
