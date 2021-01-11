
"use strict";

let ClearErr = require('./ClearErr.js')
let ConfigToolModbus = require('./ConfigToolModbus.js')
let GetAnalogIO = require('./GetAnalogIO.js')
let GetControllerDigitalIO = require('./GetControllerDigitalIO.js')
let GetDigitalIO = require('./GetDigitalIO.js')
let GetErr = require('./GetErr.js')
let GripperConfig = require('./GripperConfig.js')
let GripperMove = require('./GripperMove.js')
let GripperState = require('./GripperState.js')
let Move = require('./Move.js')
let SetAxis = require('./SetAxis.js')
let SetDigitalIO = require('./SetDigitalIO.js')
let SetInt16 = require('./SetInt16.js')
let SetLoad = require('./SetLoad.js')
let SetToolModbus = require('./SetToolModbus.js')
let TCPOffset = require('./TCPOffset.js')

module.exports = {
  ClearErr: ClearErr,
  ConfigToolModbus: ConfigToolModbus,
  GetAnalogIO: GetAnalogIO,
  GetControllerDigitalIO: GetControllerDigitalIO,
  GetDigitalIO: GetDigitalIO,
  GetErr: GetErr,
  GripperConfig: GripperConfig,
  GripperMove: GripperMove,
  GripperState: GripperState,
  Move: Move,
  SetAxis: SetAxis,
  SetDigitalIO: SetDigitalIO,
  SetInt16: SetInt16,
  SetLoad: SetLoad,
  SetToolModbus: SetToolModbus,
  TCPOffset: TCPOffset,
};
