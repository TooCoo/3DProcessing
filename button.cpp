//
// Created by Ben Eisner on 5/25/16.
//

#include <iostream>
#include "button.h"

Button::Button(int x_top_left, int y_top_left, int w, int h, const char *label, ButtonCallback callback) {
    this->x_top_left = x_top_left;
    this->y_top_left = y_top_left;
    this->w = w;
    this->h = h;
    this->label = label;
    this->callbackFunction = callback;
    this->pressState = NotPressed;
    this->hoverState = NoHover;
}

// if there's an intersection between the coordinates and the button, return true
bool Button::intersection(int x, int y) {
    return (x > x_top_left  &&  x < x_top_left + w  && y > y_top_left && y < y_top_left + h);
}

void Button::updateClickState(int x, int y, bool buttonDown) {
    if (hoverState == Hover && pressState == Pressed && !buttonDown) {
        callbackFunction();
    }

    if (intersection(x, y)) {
        pressState = buttonDown ? Pressed : NotPressed;
    }
}

bool Button::updateHoverState(int x, int y) {
    hoverState = intersection(x, y) ? Hover : NoHover;
    return (hoverState == Hover);
}
