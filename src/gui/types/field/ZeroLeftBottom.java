package gui.types.field;

import gui.FieldType;
import math.geometry.Dimension2d;

public interface ZeroLeftBottom extends FieldType {
    @Override
    default int convertY(double y, Dimension2d dimension) {
        return (int) (dimension.getY() - y);
    }
}
