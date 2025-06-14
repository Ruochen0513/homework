
"use strict";

let SetCompliancePunch = require('./SetCompliancePunch.js')
let StartController = require('./StartController.js')
let SetSpeed = require('./SetSpeed.js')
let SetComplianceMargin = require('./SetComplianceMargin.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let TorqueEnable = require('./TorqueEnable.js')
let RestartController = require('./RestartController.js')
let StopController = require('./StopController.js')
let SetTorqueLimit = require('./SetTorqueLimit.js')

module.exports = {
  SetCompliancePunch: SetCompliancePunch,
  StartController: StartController,
  SetSpeed: SetSpeed,
  SetComplianceMargin: SetComplianceMargin,
  SetComplianceSlope: SetComplianceSlope,
  TorqueEnable: TorqueEnable,
  RestartController: RestartController,
  StopController: StopController,
  SetTorqueLimit: SetTorqueLimit,
};
