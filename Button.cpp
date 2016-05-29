//
// Created by Ben Eisner on 5/25/16.
//

#include <iostream>
#include "Button.h"

Button::Button(int x_top_left, int y_top_left, int w, int h, const char *label, std::string slabel,
               ButtonCallback callback, Mouse* TheMouse) {
    this->x_top_left = x_top_left;
    this->y_top_left = y_top_left;
    this->w = w;
    this->h = h;
    this->label = label;
    this->slabel = slabel;
    this->callbackFunction = callback;
    this->TheMouse = TheMouse;
}

bool Button::ClickTest(int x, int y) {
        /*
        *	If clicked within button area, then return true
        */
        return (x > x_top_left  &&
            x < x_top_left + w  &&
            y > y_top_left      &&
            y < y_top_left + h);
}

void Button::ButtonRelease(int x, int y) {
    /*
    *	If the mouse button was pressed within the button area
    *	as well as being released on the button.....
    */
    if (ClickTest(TheMouse->xpress, TheMouse->ypress) &&
        ClickTest(x, y))
    {
        /*
        *	Then if a callback function has been set, call it.
        */
        if (callbackFunction) {
            callbackFunction();
        }
    }

    /*
    *	Set state back to zero.
    */
    state = 0;
}

void Button::ButtonPress(int x, int y) {
    /*
    *	if the mouse click was within the buttons client area,
    *	set the state to true.
    */
    if (ClickTest(x, y))
    {
        state = 1;
    }
}

bool Button::ButtonPassive(int x, int y) {
    bool output = false;

    /*
    *	if the mouse moved over the control
    */
    if (ClickTest(x, y))
    {

        /*
        *	If the cursor has just arrived over the control, set the highlighted flag
        *	and force a redraw. The screen will not be redrawn again until the mouse
        *	is no longer over this control
        */
        if (highlighted == 0) {
            highlighted = 1;
            //glutPostRedisplay();

        }

        output = true;

    }
    else

        /*
        *	If the cursor is no longer over the control, then if the control
        *	is highlighted (ie, the mouse has JUST moved off the control) then
        *	we set the highlighting back to false, and force a redraw.
        */
    if (highlighted == 1)
    {
        highlighted = 0;
        //glutPostRedisplay();
    }

    return output;
}
