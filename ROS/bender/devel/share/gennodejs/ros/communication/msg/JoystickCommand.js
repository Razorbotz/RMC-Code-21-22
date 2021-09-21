// Auto-generated. Do not edit!

// (in-package communication.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class JoystickCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.axisX = null;
      this.axisY = null;
    }
    else {
      if (initObj.hasOwnProperty('axisX')) {
        this.axisX = initObj.axisX
      }
      else {
        this.axisX = 0.0;
      }
      if (initObj.hasOwnProperty('axisY')) {
        this.axisY = initObj.axisY
      }
      else {
        this.axisY = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JoystickCommand
    // Serialize message field [axisX]
    bufferOffset = _serializer.float32(obj.axisX, buffer, bufferOffset);
    // Serialize message field [axisY]
    bufferOffset = _serializer.float32(obj.axisY, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JoystickCommand
    let len;
    let data = new JoystickCommand(null);
    // Deserialize message field [axisX]
    data.axisX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [axisY]
    data.axisY = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'communication/JoystickCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ef0f7484f7f056d7fcb586ad36e7d96';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 axisX
    float32 axisY
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JoystickCommand(null);
    if (msg.axisX !== undefined) {
      resolved.axisX = msg.axisX;
    }
    else {
      resolved.axisX = 0.0
    }

    if (msg.axisY !== undefined) {
      resolved.axisY = msg.axisY;
    }
    else {
      resolved.axisY = 0.0
    }

    return resolved;
    }
};

module.exports = JoystickCommand;
