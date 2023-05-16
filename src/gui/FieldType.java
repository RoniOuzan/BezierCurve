package gui;

import math.geometry.Dimension2d;

public interface FieldType {
    default int convertX(double x, Dimension2d dimension) {
        return (int) x;
    }

    default int convertY(double y, Dimension2d dimension) {
        return (int) y;
    }
}
