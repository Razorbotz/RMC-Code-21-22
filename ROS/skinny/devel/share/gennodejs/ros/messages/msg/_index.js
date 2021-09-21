
"use strict";

let Power = require('./Power.js');
let KeyState = require('./KeyState.js');
let VictorOut = require('./VictorOut.js');
let AxisState = require('./AxisState.js');
let HatState = require('./HatState.js');
let ButtonState = require('./ButtonState.js');
let JoystickCommand = require('./JoystickCommand.js');
let Camera = require('./Camera.js');
let TalonOut = require('./TalonOut.js');

module.exports = {
  Power: Power,
  KeyState: KeyState,
  VictorOut: VictorOut,
  AxisState: AxisState,
  HatState: HatState,
  ButtonState: ButtonState,
  JoystickCommand: JoystickCommand,
  Camera: Camera,
  TalonOut: TalonOut,
};
