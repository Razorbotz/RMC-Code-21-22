// Auto-generated. Do not edit!

// (in-package messages.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class TalonOut {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.deviceID = null;
      this.busVoltage = null;
      this.outputCurrent = null;
      this.outputVoltage = null;
      this.outputPercent = null;
      this.temperature = null;
      this.sensorPosition = null;
      this.sensorVelocity = null;
      this.closedLoopError = null;
      this.integralAccumulator = null;
      this.errorDerivative = null;
    }
    else {
      if (initObj.hasOwnProperty('deviceID')) {
        this.deviceID = initObj.deviceID
      }
      else {
        this.deviceID = 0;
      }
      if (initObj.hasOwnProperty('busVoltage')) {
        this.busVoltage = initObj.busVoltage
      }
      else {
        this.busVoltage = 0.0;
      }
      if (initObj.hasOwnProperty('outputCurrent')) {
        this.outputCurrent = initObj.outputCurrent
      }
      else {
        this.outputCurrent = 0.0;
      }
      if (initObj.hasOwnProperty('outputVoltage')) {
        this.outputVoltage = initObj.outputVoltage
      }
      else {
        this.outputVoltage = 0.0;
      }
      if (initObj.hasOwnProperty('outputPercent')) {
        this.outputPercent = initObj.outputPercent
      }
      else {
        this.outputPercent = 0.0;
      }
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = 0.0;
      }
      if (initObj.hasOwnProperty('sensorPosition')) {
        this.sensorPosition = initObj.sensorPosition
      }
      else {
        this.sensorPosition = 0;
      }
      if (initObj.hasOwnProperty('sensorVelocity')) {
        this.sensorVelocity = initObj.sensorVelocity
      }
      else {
        this.sensorVelocity = 0;
      }
      if (initObj.hasOwnProperty('closedLoopError')) {
        this.closedLoopError = initObj.closedLoopError
      }
      else {
        this.closedLoopError = 0;
      }
      if (initObj.hasOwnProperty('integralAccumulator')) {
        this.integralAccumulator = initObj.integralAccumulator
      }
      else {
        this.integralAccumulator = 0;
      }
      if (initObj.hasOwnProperty('errorDerivative')) {
        this.errorDerivative = initObj.errorDerivative
      }
      else {
        this.errorDerivative = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TalonOut
    // Serialize message field [deviceID]
    bufferOffset = _serializer.int32(obj.deviceID, buffer, bufferOffset);
    // Serialize message field [busVoltage]
    bufferOffset = _serializer.float32(obj.busVoltage, buffer, bufferOffset);
    // Serialize message field [outputCurrent]
    bufferOffset = _serializer.float32(obj.outputCurrent, buffer, bufferOffset);
    // Serialize message field [outputVoltage]
    bufferOffset = _serializer.float32(obj.outputVoltage, buffer, bufferOffset);
    // Serialize message field [outputPercent]
    bufferOffset = _serializer.float32(obj.outputPercent, buffer, bufferOffset);
    // Serialize message field [temperature]
    bufferOffset = _serializer.float32(obj.temperature, buffer, bufferOffset);
    // Serialize message field [sensorPosition]
    bufferOffset = _serializer.int32(obj.sensorPosition, buffer, bufferOffset);
    // Serialize message field [sensorVelocity]
    bufferOffset = _serializer.int32(obj.sensorVelocity, buffer, bufferOffset);
    // Serialize message field [closedLoopError]
    bufferOffset = _serializer.int32(obj.closedLoopError, buffer, bufferOffset);
    // Serialize message field [integralAccumulator]
    bufferOffset = _serializer.int32(obj.integralAccumulator, buffer, bufferOffset);
    // Serialize message field [errorDerivative]
    bufferOffset = _serializer.int32(obj.errorDerivative, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TalonOut
    let len;
    let data = new TalonOut(null);
    // Deserialize message field [deviceID]
    data.deviceID = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [busVoltage]
    data.busVoltage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [outputCurrent]
    data.outputCurrent = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [outputVoltage]
    data.outputVoltage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [outputPercent]
    data.outputPercent = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [temperature]
    data.temperature = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sensorPosition]
    data.sensorPosition = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [sensorVelocity]
    data.sensorVelocity = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [closedLoopError]
    data.closedLoopError = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [integralAccumulator]
    data.integralAccumulator = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [errorDerivative]
    data.errorDerivative = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'messages/TalonOut';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8f6fc5634dde6263b80ec7fc1aa9dee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 deviceID
    float32 busVoltage
    float32 outputCurrent
    float32 outputVoltage
    float32 outputPercent
    float32 temperature
    int32 sensorPosition
    int32 sensorVelocity
    int32 closedLoopError 
    int32 integralAccumulator
    int32 errorDerivative
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TalonOut(null);
    if (msg.deviceID !== undefined) {
      resolved.deviceID = msg.deviceID;
    }
    else {
      resolved.deviceID = 0
    }

    if (msg.busVoltage !== undefined) {
      resolved.busVoltage = msg.busVoltage;
    }
    else {
      resolved.busVoltage = 0.0
    }

    if (msg.outputCurrent !== undefined) {
      resolved.outputCurrent = msg.outputCurrent;
    }
    else {
      resolved.outputCurrent = 0.0
    }

    if (msg.outputVoltage !== undefined) {
      resolved.outputVoltage = msg.outputVoltage;
    }
    else {
      resolved.outputVoltage = 0.0
    }

    if (msg.outputPercent !== undefined) {
      resolved.outputPercent = msg.outputPercent;
    }
    else {
      resolved.outputPercent = 0.0
    }

    if (msg.temperature !== undefined) {
      resolved.temperature = msg.temperature;
    }
    else {
      resolved.temperature = 0.0
    }

    if (msg.sensorPosition !== undefined) {
      resolved.sensorPosition = msg.sensorPosition;
    }
    else {
      resolved.sensorPosition = 0
    }

    if (msg.sensorVelocity !== undefined) {
      resolved.sensorVelocity = msg.sensorVelocity;
    }
    else {
      resolved.sensorVelocity = 0
    }

    if (msg.closedLoopError !== undefined) {
      resolved.closedLoopError = msg.closedLoopError;
    }
    else {
      resolved.closedLoopError = 0
    }

    if (msg.integralAccumulator !== undefined) {
      resolved.integralAccumulator = msg.integralAccumulator;
    }
    else {
      resolved.integralAccumulator = 0
    }

    if (msg.errorDerivative !== undefined) {
      resolved.errorDerivative = msg.errorDerivative;
    }
    else {
      resolved.errorDerivative = 0
    }

    return resolved;
    }
};

module.exports = TalonOut;
