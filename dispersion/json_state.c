/* Saving bot state as json. Not for use in the real bot, only in the simulator. */
#include <kilombo.h>

#ifdef SIMULATOR

#include <jansson.h>
#include <stdio.h>
#include <string.h>
#include "dispersion.h"

json_t *json_state()
{
    // Create the state object we return
    json_t* state = json_object();

    // Store all relevant values
    // Note: make sure there are no NaN values, as they will not be saved at all by jansson !
    // ...

    return state;
}

#endif
// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
