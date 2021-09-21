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

class HatState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joystick = null;
      this.hat = null;
      this.state = null;
    }
    else {
      if (initObj.hasOwnProperty('joystick')) {
        this.joystick = initObj.joystick
      }
      else {
        this.joystick = 0;
      }
      if (initObj.hasOwnProperty('hat')) {
        this.hat = initObj.hat
      }
      else {
        this.hat = 0;
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HatState
    // Serialize message field [joystick]
    bufferOffset = _serializer.uint8(obj.joystick, buffer, bufferOffset);
    // Serialize message field [hat]
    bufferOffset = _serializer.uint8(obj.hat, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HatState
    let len;
    let data = new HatState(null);
    // Deserialize message field [joystick]
    data.joystick = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [hat]
    data.hat = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a message object
    return 'messages/HatState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b0f14c98250145be7590c9d7a97bb21b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 joystick
    uint8 hat
    uint8 state
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HatState(null);
    if (msg.joystick !== undefined) {
      resolved.joystick = msg.joystick;
    }
    else {
      resolved.joystick = 0
    }

    if (msg.hat !== undefined) {
      resolved.hat = msg.hat;
    }
    else {
      resolved.hat = 0
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    return resolved;
    }
};

module.exports = HatState;
