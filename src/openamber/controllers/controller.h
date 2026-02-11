#pragma once

#include "esphome.h"

using namespace esphome;

class Controller
{
public:
  virtual ~Controller() = default;
  virtual bool HasDemand() { return false; }
  virtual void OnCompressorStarted() {}
  virtual float GetPreferredPumpSpeed() { return 0.0f; }
  virtual bool ShouldWaitForTemperatureStabilizationBeforeCompressorStart() { return true; }
  virtual void StartTrackingTemperatureChanges() {}
  virtual bool ShouldSoftStart() { return true; }
  virtual bool ShouldActivateBackupHeater() { return false; }
  virtual int DetermineCompressorMode() { return 0; }
  virtual bool ShouldStopCompressor() { return false; }
};