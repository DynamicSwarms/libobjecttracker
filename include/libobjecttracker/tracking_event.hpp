#pragma once

#include <functional>

namespace libobjecttracker {

class Object;

enum class TrackingEvent
{
  acquired,
  initialization_timed_out,
  lost
};

using TrackingEventCallback =
  std::function<void(TrackingEvent, const Object&)>;

}  // namespace libobjecttracker
