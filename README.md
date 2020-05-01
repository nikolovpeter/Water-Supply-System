# Water-Supply-System based on Arduino MKR WiFi 1010.
Autonomous water supply of a small house in a remote location by a drill well. Water is pumped from the well and stored in a local water tank in the house, and from there supplied to the house by a pressure pump.
The system monitors water level (using a laser distance sensor), several temperatures, filling pump water flow, filling pump AC current, and controls two pumps - a filling pump (in the well) and a pressure pump (supplying the house with water). The filling pump is directly operated via a relay, and the pressure pump is allowed or disallowed via a relay.
The parameters and status be controlled via several channels:
  - Local user interface with screen and rotary encoder;
  - Thinger IOT cloud;
  - Telegram bot messages.
