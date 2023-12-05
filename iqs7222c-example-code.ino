/******************************************************************************
 *                                                                            *
 *                                Copyright by                                *
 *                                                                            *
 *                              Azoteq (Pty) Ltd                              *
 *                          Republic of South Africa                          *
 *                                                                            *
 *                           Tel: +27(0)21 863 0033                           *
 *                           E-mail: info@azoteq.com                          *
 *                                                                            *
 * ========================================================================== *
 * @file        iqs7222c-example-code.ino                                     *
 * @brief       IQS7222C Buttoned Wheel EV-Kit Example code                   *
 *              (PCB Version - AZP1192A2)                                     *
 *              This example demonstrates how to write the desired settings   *
 *              to the IQS7222C to use the Buttoned Wheel EV-Kit              *
 *                                                                            *
 *              All data is displayed over serial communication with the      *
 *              following outputs:                                            *
 *                - Proximity and touch detection on the 4 buttons            *
 *                  (Channel 6,7,8,9)                                         *
 *                - Wheel Coordinate output                                   *
 *                - Power Mode Switching (if enabled)                         *
 *                - Force Communication when 'f\n' is sent over Serial        *
 *                - Software Reset when 'r\n' is sent over Serial             *
 * @author      Azoteq PTY Ltd                                                *
 * @version     v1.1                                                          *
 * @date        2023-07-10                                                    *
 ******************************************************************************/

#include <Arduino.h>
#include "src\IQS7222C\IQS7222C.h"

/*** Defines ***/
#define DEMO_IQS7222C_ADDR                     0x44
#define DEMO_IQS7222C_POWER_PIN                4
#define DEMO_IQS7222C_RDY_PIN                  7

/*** Instances ***/
IQS7222C iqs7222c;

/*** Global Variables ***/
iqs7222c_ch_states button_states[4] = {IQS7222C_CH_NONE, IQS7222C_CH_NONE, IQS7222C_CH_NONE, IQS7222C_CH_NONE};
iqs7222c_power_modes power_mode = IQS7222C_NORMAL_POWER;
uint16_t slider_position = 65535;
bool show_data = false;

void setup()
{
  /* Start Serial Communication */
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Start Serial communication");
  delay(200);

  /* Power On IQS7222C */
  pinMode(DEMO_IQS7222C_POWER_PIN, OUTPUT);
  delay(200);
  digitalWrite(DEMO_IQS7222C_POWER_PIN, LOW);
  delay(200);
  digitalWrite(DEMO_IQS7222C_POWER_PIN, HIGH);

  /* Initialize the IQS7222C with input parameters device address and RDY pin */
  iqs7222c.begin(DEMO_IQS7222C_ADDR, DEMO_IQS7222C_RDY_PIN);
  Serial.println("IQS7222C Ready");
  delay(1);
}

void loop()
{
  iqs7222c.run(); // Runs the IQS7222C

  force_comms_and_reset(); // function to initialize a force communication window.

  /* Process data read from IQS7222C when new data is available (RDY Line Low) */
  if(iqs7222c.new_data_available)
  {
    check_power_mode();       // Verify if a power mode change occurred
    read_slider_coordinates();// Read the latest slider coordinates
    check_channel_states();   // Check if a channel state change has occurred
    show_iqs7222c_data();     // Show data if a forced communication window was requested

    iqs7222c.new_data_available = false;
  }
}

/* Function to check when the Current power mode of the IQS7222C changed */
void check_power_mode(void)
{
  iqs7222c_power_modes buffer = iqs7222c.get_PowerMode(); // read current power mode

  if(buffer != power_mode)
  {
    switch(buffer)
    {
      case IQS7222C_NORMAL_POWER:
        Serial.println("NORMAL POWER Activated!");
        break;
      case IQS7222C_LOW_POWER:
        Serial.println("LOW POWER Activated!");
        break;
      case IQS7222C_ULP:
        Serial.println("ULP Activated!");
        break;
    }

    /* Update the running state */
    power_mode = buffer;
  }
}

/* Function to check the Proximity and Touch states of the self-capacitance buttons */
void check_channel_states(void)
{
  iqs7222c_channel_e ch = IQS7222C_CH6;
  /* Loop through all the button channels */
  for(uint8_t i = 0; i < 4; i++)
  {
    /* check if the touch state bit is set */
    if(iqs7222c.channel_touchState((iqs7222c_channel_e)(ch+i)))
    {
      if(button_states[i] != IQS7222C_CH_TOUCH)
      {
        Serial.print("CH");
        Serial.print(i+6);
        Serial.println(": Touch");
        button_states[i] = IQS7222C_CH_TOUCH;
      }
    }
    /* check if the proximity state bit is set */
    else if (iqs7222c.channel_proxState((iqs7222c_channel_e)(ch+i)))
    {
      if(button_states[i] != IQS7222C_CH_PROX)
      {
        Serial.print("CH");
        Serial.print(i+6);
        Serial.println(": Prox");
        button_states[i] = IQS7222C_CH_PROX;
      }
    }
    /* IQS7222C_CH_NONE state if neither the touch nor proximity bit was set to 1 */
    else
    {
      if(button_states[i] != IQS7222C_CH_NONE)
      {
        Serial.print("CH");
        Serial.print(i+6);
        Serial.println(": None");
        button_states[i] = IQS7222C_CH_NONE;
      }
    }
  }
}

/* Read the current coordinate of a finger on Slider 0 */
void read_slider_coordinates(void)
{
  /* read slider coordinates from memory */
  uint16_t buffer = iqs7222c.sliderCoordinate(IQS7222C_SLIDER0);

  if(buffer != slider_position)
  {
    if(slider_position == 65535)
    {
      Serial.println("----- SLIDER 0 Start -----");
    }
    slider_position = buffer;
    if(slider_position == 65535)
    {
      Serial.println("------ SLIDER 0 End ------");
    }
    else
    {
      Serial.print("SLIDER 0 Coordinate: ");
      Serial.println(slider_position);
    }
  }
}

/* Force the IQS7222C to open a RDY window and read the current state of the
device or request a software reset */
void force_comms_and_reset(void)
{
  char message = read_message();

  /* If an 'f' was received over serial, open a forced communication window and
  print the new data received */
  if(message == 'f')
  {
    iqs7222c.force_I2C_communication(); // prompt the IQS7222C
    show_data = true;
  }

  /* If an 'r' was received over serial, request a software(SW) reset */
  if(message == 'r')
  {
    Serial.println("Software Reset Requested!");
    iqs7222c.force_I2C_communication(); // Request a RDY window
    iqs7222c.iqs7222c_state.state = IQS7222C_STATE_SW_RESET;
    power_mode = IQS7222C_NORMAL_POWER;
  }
}

/* Read message sent over serial communication */
char read_message(void)
{
  while (Serial.available())
  {
    if(Serial.available() > 0)
    {
      return  Serial.read();
    }
  }

  return '\n';
}

/* Shows the current IQS7222C data */
void show_iqs7222c_data()
{
  if(show_data)
  {
    Serial.println("******** IQS7222C DATA *********");
    button_states[0] = IQS7222C_CH_UNKNOWN;
    button_states[1] = IQS7222C_CH_UNKNOWN;
    button_states[2] = IQS7222C_CH_UNKNOWN;
    button_states[3] = IQS7222C_CH_UNKNOWN;
    check_channel_states();
    Serial.print("SLIDER 0 Coordinate: ");
    Serial.println(iqs7222c.sliderCoordinate(IQS7222C_SLIDER0));
    power_mode = IQS7222C_POWER_UNKNOWN;
    check_power_mode();
    Serial.println("********************************");
    show_data = false;
  }
}
