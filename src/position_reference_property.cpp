/* Copyright 2014 Gareth Cross, 2018-2019 TomTom N.V.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */

#include "position_reference_property.h"

namespace rviz
{
const QString PositionReferenceProperty::FIX_MSG_STRING = "<NavSatFix Message>";

PositionReferenceProperty::PositionReferenceProperty(
    const QString& name, const QString& default_value, const QString& description, Property* parent,
    FrameManager* frame_manager, const char* changed_slot, QObject* receiver)
  : TfFrameProperty(name, default_value, description, parent, frame_manager, true, changed_slot, receiver)
{
  connect(this, SIGNAL(requestOptions(EditableEnumProperty*)), this, SLOT(adjustOptionsList()));
}

void PositionReferenceProperty::adjustOptionsList()
{
  // This function expects that TfFrameProperty::fillFrameList() has already run
  strings_.push_front(FIX_MSG_STRING);
}

QString PositionReferenceProperty::getFrame() const
{
  const auto& frame = TfFrameProperty::getFrame();
  if (frame == FIX_MSG_STRING)
    return "";
  return frame;
}

std::string PositionReferenceProperty::getFrameStd() const
{
  return getFrame().toStdString();
}

PositionReferenceType PositionReferenceProperty::getPositionReferenceType() const
{
  if (getValue().toString() == FIX_MSG_STRING)
  {
    return PositionReferenceType::NAV_SAT_FIX_MESSAGE;
  }
  else
  {
    return PositionReferenceType::TF_FRAME;
  }
}

}
