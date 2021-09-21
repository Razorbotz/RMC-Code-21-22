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

class AxisState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joystick = null;
      this.axis = null;
      this.state = null;
    }
    else {
      if (initObj.hasOwnProperty('joystick')) {
        this.joystick = initObj.joystick
      }
      else {
        this.joystick = 0;
      }
      if (initObj.hasOwnProperty('axis')) {
        this.axis = initObj.axis
      }
      else {
        this.axis = 0;
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AxisState
    // Serialize message field [joystick]
    bufferOffset = _serializer.uint8(obj.joystick, buffer, bufferOffset);
    // Serialize message field [axis]
    bufferOffset = _serializer.uint8(obj.axis, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.float32(obj.state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AxisState
    let len;
    let data = new AxisState(null);
    // Deserialize message field [joystick]
    data.joystick = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [axis]
    data.axis = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'messages/AxisState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '66b25cf96600974c62d570e9935b51ff';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 joystick
    uint8 axis
    float32 state
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AxisState(null);
    if (msg.joystick !== undefined) {
      resolved.joystick = msg.joystick;
    }
    else {
      resolved.joystick = 0
    }

    if (msg.axis !== undefined) {
      resolved.axis = msg.axis;
    }
    else {
      resolved.axis = 0
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0.0
    }

    return resolved;
    }
};

module.exports = AxisState;
