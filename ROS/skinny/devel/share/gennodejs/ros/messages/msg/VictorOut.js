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

class VictorOut {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.deviceID = null;
      this.busVoltage = null;
      this.outputVoltage = null;
      this.outputPercent = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VictorOut
    // Serialize message field [deviceID]
    bufferOffset = _serializer.int32(obj.deviceID, buffer, bufferOffset);
    // Serialize message field [busVoltage]
    bufferOffset = _serializer.float32(obj.busVoltage, buffer, bufferOffset);
    // Serialize message field [outputVoltage]
    bufferOffset = _serializer.float32(obj.outputVoltage, buffer, bufferOffset);
    // Serialize message field [outputPercent]
    bufferOffset = _serializer.float32(obj.outputPercent, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VictorOut
    let len;
    let data = new VictorOut(null);
    // Deserialize message field [deviceID]
    data.deviceID = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [busVoltage]
    data.busVoltage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [outputVoltage]
    data.outputVoltage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [outputPercent]
    data.outputPercent = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'messages/VictorOut';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1308269e2bd460ac3f55ed9f4bcd8d51';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 deviceID
    float32 busVoltage
    float32 outputVoltage
    float32 outputPercent
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VictorOut(null);
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

    return resolved;
    }
};

module.exports = VictorOut;
