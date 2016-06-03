//
// Created by Ben Eisner on 5/25/16.
//

#ifndef INC_3D_PROCESSING_BUTTON_H
#define INC_3D_PROCESSING_BUTTON_H

#include <string>
#include "enums.h"

typedef void(*ButtonCallback)();

struct TextBox {
    int x_top_left;
    int y_top_left;
    int w;
    int h;
    const char* label;
};

typedef struct TextBox TextBox;

// ----------------------- mouse functionality---------------------------------
// Based on code found at: https://nccastaff.bournemouth.ac.uk/jmacey/RobTheBloke/www/opengl_programming.html
// Modifed - some copied - I assume that's fine since I'm not being assessed on my GL Buttons and mouse clicks
struct Mouse
{
    int x;		/*	the x coordinate of the mouse cursor	*/
    int y;		/*	the y coordinate of the mouse cursor	*/
    int lmb;	/*	is the left button pressed?		*/
    int mmb;	/*	is the middle button pressed?	*/
    int rmb;	/*	is the right button pressed?	*/

    int dx;     /* delta x */
    int dy;     /* delta y */

    int xpress; /*	stores the x-coord of when the first button press occurred	*/
    int ypress; /*	stores the y-coord of when the first button press occurred	*/
};

typedef struct Mouse Mouse;

// ---------------------- Button interaction System -------------------------
// Based on code found at: https://nccastaff.bournemouth.ac.uk/jmacey/RobTheBloke/www/opengl_programming.html
// Modified for my use
class Button {
private:

    ButtonCallback callbackFunction;

    bool intersection(int x, int y) {
        return (x > x_top_left  &&  x < x_top_left + w  && y > y_top_left && y < y_top_left + h);
    }

public:
    int x_top_left;
    int y_top_left;

    int w;				    // width
    int h;				    // height
    const char* label;		// button text

    PressState pressState;
    HoverState hoverState;

    Button(int x_top_left, int y_top_left, int w, int h, const char *label, ButtonCallback callback) {
        this->x_top_left = x_top_left;
        this->y_top_left = y_top_left;
        this->w = w;
        this->h = h;
        this->label = label;
        this->callbackFunction = callback;
        this->pressState = NotPressed;
        this->hoverState = NoHover;
    }

    void updateClickState(int x, int y, bool buttonDown) {
        if (hoverState == Hover && pressState == Pressed && !buttonDown) {
            callbackFunction();
        }

        if (intersection(x, y)) {
            pressState = buttonDown ? Pressed : NotPressed;
        }
    }

    bool updateHoverState(int x, int y) {
        hoverState = intersection(x, y) ? Hover : NoHover;
        return (hoverState == Hover);
    }

};

#endif //INC_3D_PROCESSING_BUTTON_H
