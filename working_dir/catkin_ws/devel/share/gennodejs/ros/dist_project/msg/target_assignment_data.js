// Auto-generated. Do not edit!

// (in-package dist_project.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class target_assignment_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sender_robot_id = null;
      this.assigned_robot_id = null;
      this.x = null;
      this.y = null;
      this.z = null;
    }
    else {
      if (initObj.hasOwnProperty('sender_robot_id')) {
        this.sender_robot_id = initObj.sender_robot_id
      }
      else {
        this.sender_robot_id = 0;
      }
      if (initObj.hasOwnProperty('assigned_robot_id')) {
        this.assigned_robot_id = initObj.assigned_robot_id
      }
      else {
        this.assigned_robot_id = 0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type target_assignment_data
    // Serialize message field [sender_robot_id]
    bufferOffset = _serializer.int64(obj.sender_robot_id, buffer, bufferOffset);
    // Serialize message field [assigned_robot_id]
    bufferOffset = _serializer.int64(obj.assigned_robot_id, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type target_assignment_data
    let len;
    let data = new target_assignment_data(null);
    // Deserialize message field [sender_robot_id]
    data.sender_robot_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [assigned_robot_id]
    data.assigned_robot_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dist_project/target_assignment_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e421d61aefbe78f0d6c0c39a64cfb320';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 sender_robot_id
    int64 assigned_robot_id
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new target_assignment_data(null);
    if (msg.sender_robot_id !== undefined) {
      resolved.sender_robot_id = msg.sender_robot_id;
    }
    else {
      resolved.sender_robot_id = 0
    }

    if (msg.assigned_robot_id !== undefined) {
      resolved.assigned_robot_id = msg.assigned_robot_id;
    }
    else {
      resolved.assigned_robot_id = 0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    return resolved;
    }
};

module.exports = target_assignment_data;
