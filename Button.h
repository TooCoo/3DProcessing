//
// Created by Ben Eisner on 5/25/16.
//

#ifndef INC_3D_PROCESSING_BUTTON_H
#define INC_3D_PROCESSING_BUTTON_H

#include <string>
#include "mouse.h"
#include "enums.h"

// ---------------------- Button interaction System -------------------------
// Based on code found at: https://nccastaff.bournemouth.ac.uk/jmacey/RobTheBloke/www/opengl_programming.html
// Modified for my use
typedef void(*ButtonCallback)();

//struct for button
class Button
{
public:
    int x_top_left;
    int y_top_left;

    int w;				    // width
    int h;				    // height
    const char* label;		// button text
    ButtonCallback callbackFunction;

    PressState pressState;
    HoverState hoverState;

    bool intersection(int x, int y);

    Button(int x_top_left, int y_top_left, int w, int h, const char *label, ButtonCallback callback);
    void updateClickState(int x, int y, bool buttonDown);
    bool updateHoverState(int x, int y);

};

#endif //INC_3D_PROCESSING_BUTTON_H
