//
// Created by Ben Eisner on 5/28/16.
//

#ifndef INC_3D_PROCESSING_MOUSE_H
#define INC_3D_PROCESSING_MOUSE_H

// ----------------------- Mouse functionality---------------------------------
// Based on code found at: https://nccastaff.bournemouth.ac.uk/jmacey/RobTheBloke/www/opengl_programming.html
// Modifed - some copied - I assume that's fine since I'm not being assessed on my GL Buttons and Mouse clicks
struct Mouse
{
    int x;		/*	the x coordinate of the mouse cursor	*/
    int y;		/*	the y coordinate of the mouse cursor	*/
    int lmb;	/*	is the left button pressed?		*/
    int mmb;	/*	is the middle button pressed?	*/
    int rmb;	/*	is the right button pressed?	*/

    int dx;
    int dy;

    int xpress; /*	stores the x-coord of when the first button press occurred	*/
    int ypress; /*	stores the y-coord of when the first button press occurred	*/
};

#endif //INC_3D_PROCESSING_MOUSE_H
