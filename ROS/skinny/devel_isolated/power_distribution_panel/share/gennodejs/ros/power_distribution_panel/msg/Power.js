// Auto-generated. Do not edit!

// (in-package power_distribution_panel.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Power {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.voltage = null;
      this.current0 = null;
      this.current1 = null;
      this.current2 = null;
      this.current3 = null;
      this.current4 = null;
      this.current5 = null;
      this.current6 = null;
      this.current7 = null;
      this.current8 = null;
      this.current9 = null;
      this.current10 = null;
      this.current11 = null;
      this.current12 = null;
      this.current13 = null;
      this.current14 = null;
      this.current15 = null;
    }
    else {
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = 0.0;
      }
      if (initObj.hasOwnProperty('current0')) {
        this.current0 = initObj.current0
      }
      else {
        this.current0 = 0.0;
      }
      if (initObj.hasOwnProperty('current1')) {
        this.current1 = initObj.current1
      }
      else {
        this.current1 = 0.0;
      }
      if (initObj.hasOwnProperty('current2')) {
        this.current2 = initObj.current2
      }
      else {
        this.current2 = 0.0;
      }
      if (initObj.hasOwnProperty('current3')) {
        this.current3 = initObj.current3
      }
      else {
        this.current3 = 0.0;
      }
      if (initObj.hasOwnProperty('current4')) {
        this.current4 = initObj.current4
      }
      else {
        this.current4 = 0.0;
      }
      if (initObj.hasOwnProperty('current5')) {
        this.current5 = initObj.current5
      }
      else {
        this.current5 = 0.0;
      }
      if (initObj.hasOwnProperty('current6')) {
        this.current6 = initObj.current6
      }
      else {
        this.current6 = 0.0;
      }
      if (initObj.hasOwnProperty('current7')) {
        this.current7 = initObj.current7
      }
      else {
        this.current7 = 0.0;
      }
      if (initObj.hasOwnProperty('current8')) {
        this.current8 = initObj.current8
      }
      else {
        this.current8 = 0.0;
      }
      if (initObj.hasOwnProperty('current9')) {
        this.current9 = initObj.current9
      }
      else {
        this.current9 = 0.0;
      }
      if (initObj.hasOwnProperty('current10')) {
        this.current10 = initObj.current10
      }
      else {
        this.current10 = 0.0;
      }
      if (initObj.hasOwnProperty('current11')) {
        this.current11 = initObj.current11
      }
      else {
        this.current11 = 0.0;
      }
      if (initObj.hasOwnProperty('current12')) {
        this.current12 = initObj.current12
      }
      else {
        this.current12 = 0.0;
      }
      if (initObj.hasOwnProperty('current13')) {
        this.current13 = initObj.current13
      }
      else {
        this.current13 = 0.0;
      }
      if (initObj.hasOwnProperty('current14')) {
        this.current14 = initObj.current14
      }
      else {
        this.current14 = 0.0;
      }
      if (initObj.hasOwnProperty('current15')) {
        this.current15 = initObj.current15
      }
      else {
        this.current15 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Power
    // Serialize message field [voltage]
    bufferOffset = _serializer.float32(obj.voltage, buffer, bufferOffset);
    // Serialize message field [current0]
    bufferOffset = _serializer.float32(obj.current0, buffer, bufferOffset);
    // Serialize message field [current1]
    bufferOffset = _serializer.float32(obj.current1, buffer, bufferOffset);
    // Serialize message field [current2]
    bufferOffset = _serializer.float32(obj.current2, buffer, bufferOffset);
    // Serialize message field [current3]
    bufferOffset = _serializer.float32(obj.current3, buffer, bufferOffset);
    // Serialize message field [current4]
    bufferOffset = _serializer.float32(obj.current4, buffer, bufferOffset);
    // Serialize message field [current5]
    bufferOffset = _serializer.float32(obj.current5, buffer, bufferOffset);
    // Serialize message field [current6]
    bufferOffset = _serializer.float32(obj.current6, buffer, bufferOffset);
    // Serialize message field [current7]
    bufferOffset = _serializer.float32(obj.current7, buffer, bufferOffset);
    // Serialize message field [current8]
    bufferOffset = _serializer.float32(obj.current8, buffer, bufferOffset);
    // Serialize message field [current9]
    bufferOffset = _serializer.float32(obj.current9, buffer, bufferOffset);
    // Serialize message field [current10]
    bufferOffset = _serializer.float32(obj.current10, buffer, bufferOffset);
    // Serialize message field [current11]
    bufferOffset = _serializer.float32(obj.current11, buffer, bufferOffset);
    // Serialize message field [current12]
    bufferOffset = _serializer.float32(obj.current12, buffer, bufferOffset);
    // Serialize message field [current13]
    bufferOffset = _serializer.float32(obj.current13, buffer, bufferOffset);
    // Serialize message field [current14]
    bufferOffset = _serializer.float32(obj.current14, buffer, bufferOffset);
    // Serialize message field [current15]
    bufferOffset = _serializer.float32(obj.current15, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Power
    let len;
    let data = new Power(null);
    // Deserialize message field [voltage]
    data.voltage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current0]
    data.current0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current1]
    data.current1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current2]
    data.current2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current3]
    data.current3 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current4]
    data.current4 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current5]
    data.current5 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current6]
    data.current6 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current7]
    data.current7 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current8]
    data.current8 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current9]
    data.current9 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current10]
    data.current10 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current11]
    data.current11 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current12]
    data.current12 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current13]
    data.current13 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current14]
    data.current14 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current15]
    data.current15 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 68;
  }

  static datatype() {
    // Returns string type for a message object
    return 'power_distribution_panel/Power';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a5bf87df7460476e356f8a0cdbf06096';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 voltage
    float32 current0
    float32 current1
    float32 current2
    float32 current3
    float32 current4
    float32 current5
    float32 current6
    float32 current7
    float32 current8
    float32 current9
    float32 current10
    float32 current11
    float32 current12
    float32 current13
    float32 current14 
    float32 current15
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Power(null);
    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = 0.0
    }

    if (msg.current0 !== undefined) {
      resolved.current0 = msg.current0;
    }
    else {
      resolved.current0 = 0.0
    }

    if (msg.current1 !== undefined) {
      resolved.current1 = msg.current1;
    }
    else {
      resolved.current1 = 0.0
    }

    if (msg.current2 !== undefined) {
      resolved.current2 = msg.current2;
    }
    else {
      resolved.current2 = 0.0
    }

    if (msg.current3 !== undefined) {
      resolved.current3 = msg.current3;
    }
    else {
      resolved.current3 = 0.0
    }

    if (msg.current4 !== undefined) {
      resolved.current4 = msg.current4;
    }
    else {
      resolved.current4 = 0.0
    }

    if (msg.current5 !== undefined) {
      resolved.current5 = msg.current5;
    }
    else {
      resolved.current5 = 0.0
    }

    if (msg.current6 !== undefined) {
      resolved.current6 = msg.current6;
    }
    else {
      resolved.current6 = 0.0
    }

    if (msg.current7 !== undefined) {
      resolved.current7 = msg.current7;
    }
    else {
      resolved.current7 = 0.0
    }

    if (msg.current8 !== undefined) {
      resolved.current8 = msg.current8;
    }
    else {
      resolved.current8 = 0.0
    }

    if (msg.current9 !== undefined) {
      resolved.current9 = msg.current9;
    }
    else {
      resolved.current9 = 0.0
    }

    if (msg.current10 !== undefined) {
      resolved.current10 = msg.current10;
    }
    else {
      resolved.current10 = 0.0
    }

    if (msg.current11 !== undefined) {
      resolved.current11 = msg.current11;
    }
    else {
      resolved.current11 = 0.0
    }

    if (msg.current12 !== undefined) {
      resolved.current12 = msg.current12;
    }
    else {
      resolved.current12 = 0.0
    }

    if (msg.current13 !== undefined) {
      resolved.current13 = msg.current13;
    }
    else {
      resolved.current13 = 0.0
    }

    if (msg.current14 !== undefined) {
      resolved.current14 = msg.current14;
    }
    else {
      resolved.current14 = 0.0
    }

    if (msg.current15 !== undefined) {
      resolved.current15 = msg.current15;
    }
    else {
      resolved.current15 = 0.0
    }

    return resolved;
    }
};

module.exports = Power;
