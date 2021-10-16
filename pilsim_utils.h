#pragma once
#include "Arduino.h"
#include "Filters.h"
#include "SerialTransfer.h"
#include "Autopilot.h"




const int JOY_MAX            = 1023;
const int JOY_MIN            = 0;
const int GEAR_TOGGLE_BUTTON = 1;
const int FLAP_TOGGLE_BUTTON = 2;
const int PAYL_TOGGLE_BUTTON = 3;

const byte NAV       = 0;
const byte NAV_DUBIN = 1;
const byte LOITER    = 2;
const byte LAUNCH    = 3;
const byte LAND      = 4;
const byte SPECIAL   = 5;




pilsim_state_params prevPlane;
pilsim_state_params plane;

SerialTransfer feedback;

FilterOnePole lowpassFilter(LOWPASS, 0.2);

orbit_dir dir    = CLOCKWISE;
dubin_mode dMode = TURN_I;
nav_frame nFrame;
navigator nav;




double throttleCommand = JOY_MAX * 0.85;
double pitchCommand    = 512;
double rollCommand     = 512;
double yawCommand      = 512;

double angleOfClimb = 0;

int prevState = LAUNCH;
int curState  = LAUNCH;




bool handleData()
{
  if(feedback.available())
  {
    prevPlane = plane;
    feedback.rxObj(plane);
    angleOfClimb = lowpassFilter.input(climbAngle(prevPlane.lat, prevPlane.lon, plane.lat, plane.lon, prevPlane.alt, plane.alt));
    
    return true;
  }

  return false;
}




void toggleButton(const int& btnNum)
{
  Joystick.button(btnNum, 1);
  delay(10);
  Joystick.button(btnNum, 0);
}




void toggleGear()
{
  toggleButton(GEAR_TOGGLE_BUTTON);
}




void toggleFlaps()
{
  toggleButton(FLAP_TOGGLE_BUTTON);
}




void togglePayload()
{
  toggleButton(PAYL_TOGGLE_BUTTON);
}




bool drop(const double& planeLat, const double& planeLon, const double& planeAgl, const double& planeIas, const double& targetLat, const double& targetLon)
{
  double distA = distance(planeLat, planeLon, targetLat, targetLon);
  double t     = sqrt((2 * planeAgl) / EARTH_GRAVITY);
  double distB = (planeIas / 3.6) * t;

  if (abs(distA - distB) < 5)
    return true;
  return false;
}




void sendJoyCommands()
{
  Joystick.X(pitchCommand);
  Joystick.Y(rollCommand);
  Joystick.Z(yawCommand);
  Joystick.Zrotate(throttleCommand);
}
